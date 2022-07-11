// SPDX-License-Identifier: GPL-2.0
/**
 * Alif Video Platform device driver
 *
 * Copyright (C) 2022 Alif Semicondutor.
 * Author: Harith George <harith.g@alifsemi.com>
 *
 * Based on DWC Video Platform device driver from Synopsys
 */

#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-vmalloc.h>
#include <media/v4l2-subdev.h>
#include <media/dwc/dw-mipi-csi-pltfrm.h>

#include "plat.h"
#include "cpi.h"

static int plat_csi_pipeline_s_format(struct plat_csi_media_pipeline *ep,
				      struct v4l2_subdev_format *fmt)
{
	struct plat_csi_pipeline *p = to_plat_csi_pipeline(ep);
	static const u8 seq[IDX_MAX] = {IDX_SENSOR1, IDX_SENSOR2, IDX_CSI,
					IDX_CPI, IDX_VDEV};

	fmt->which = V4L2_SUBDEV_FORMAT_ACTIVE;
	v4l2_subdev_call(p->subdevs[seq[IDX_CSI]], pad, set_fmt, NULL, fmt);

	return 0;
}

static void plat_csi_pipeline_prepare(struct plat_csi_pipeline *p,
				      struct media_entity *me)
{
	struct v4l2_subdev *sd;
	unsigned int i = 0;

	for (i = 0; i < IDX_MAX; i++)
		p->subdevs[i] = NULL;

	while (1) {
		struct media_pad *pad = NULL;

		/* Find remote source pad */
		for (i = 0; i < me->num_pads; i++) {
			struct media_pad *spad = &me->pads[i];

			if (!(spad->flags & MEDIA_PAD_FL_SINK))
				continue;

			pad = media_entity_remote_pad(spad);
			if (pad) {

				sd = media_entity_to_v4l2_subdev(pad->entity);
				if (sd->grp_id == GRP_ID_CPI) {
						break;
				}
			}
		}
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;
		switch (sd->grp_id) {
		case GRP_ID_SENSOR:
			p->subdevs[IDX_SENSOR1] = sd;
			break;
		case GRP_ID_SENSOR + 1:
			p->subdevs[IDX_SENSOR2] = sd;
			break;
		case GRP_ID_CSI:
			p->subdevs[IDX_CSI] = sd;
			break;
		case GRP_ID_CPI:
			p->subdevs[IDX_CPI] = sd;
			break;
		case GRP_ID_VIDEODEV:
			p->subdevs[IDX_VDEV] = sd;
			break;
		default:
			break;
		}
		me = &sd->entity;
		if (me->num_pads == 1)
			break;
	}
}

static int subdev_set_power(struct v4l2_subdev *sd, int on)
{
	int *use_count;
	int ret;

	if (sd == NULL) {
		pr_info("null subdev\n");
		return -ENXIO;
	}
	use_count = &sd->entity.use_count;
	if (on && (*use_count)++ > 0)
		return 0;
	else if (!on && (*use_count == 0 || --(*use_count) > 0))
		return 0;

	pr_info("%s %s !\n", on ? "on" : "off", sd->entity. name);
	ret = v4l2_subdev_call(sd, core, s_power, on);

	return ret != -ENOIOCTLCMD ? ret : 0;
}

static int plat_csi_pipeline_s_power(struct plat_csi_pipeline *p, bool on)
{
	static const u8 seq[IDX_MAX] = {IDX_CSI, IDX_SENSOR1,
					IDX_CPI, IDX_SENSOR2,
					IDX_VDEV};
	int i, idx, ret = 0;

	for (i = 0; i < IDX_MAX; i++) {
		idx = seq[i];

		if (p->subdevs[idx] == NULL)
			pr_debug("No device registered on %d\n", idx);
		else {
			ret = subdev_set_power(p->subdevs[idx], on);
			if (ret < 0 && ret != -ENXIO)
				goto error;
		}
	}
	return 0;
error:
	for (; i >= 0; i--) {
		idx = seq[i];

		subdev_set_power(p->subdevs[idx], !on);
	}
	return ret;
}

static int plat_csi_pipeline_open(struct plat_csi_media_pipeline *ep,
				  struct media_entity *me,
				  bool prepare)
{
	struct plat_csi_pipeline *p = to_plat_csi_pipeline(ep);
	int ret;

	if (WARN_ON(p == NULL || me == NULL))
		return -EINVAL;

	if (prepare)
		plat_csi_pipeline_prepare(p, me);

	ret = plat_csi_pipeline_s_power(p, 1);
	if (!ret)
		return 0;

	return ret;
}

static int plat_csi_pipeline_close(struct plat_csi_media_pipeline *ep)
{
	struct plat_csi_pipeline *p = to_plat_csi_pipeline(ep);
	int ret;

	ret = plat_csi_pipeline_s_power(p, 0);

	return ret == -ENXIO ? 0 : ret;
}

static int plat_csi_pipeline_s_stream(struct plat_csi_media_pipeline *ep,
				 bool on)
{
	static const u8 seq[IDX_MAX] = {IDX_VDEV, IDX_CPI, IDX_CSI,
					IDX_SENSOR1, IDX_SENSOR2};
	struct plat_csi_pipeline *p = to_plat_csi_pipeline(ep);
	int i, ret = 0,	idx;

	for (i = 0; i < IDX_MAX; i++) {
		idx = seq[i];

		if (p->subdevs[idx] == NULL)
			pr_debug("No device registered on %d\n", idx);
		else {
			ret = v4l2_subdev_call(p->subdevs[idx], video, s_stream, on);

			if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
				goto error;
		}
	}
	return 0;
error:
	for (; i >= 0; i--) {
		idx = seq[i];
		v4l2_subdev_call(p->subdevs[idx], video, s_stream, !on);
	}
	return ret;
}

/* Media pipeline operations for the FIMC/FIMC-LITE video device driver */
static const struct plat_csi_media_pipeline_ops plat_csi_pipeline_ops = {
	.open		= plat_csi_pipeline_open,
	.close		= plat_csi_pipeline_close,
	.set_format	= plat_csi_pipeline_s_format,
	.set_stream	= plat_csi_pipeline_s_stream,
};

static struct
plat_csi_media_pipeline *plat_csi_pipeline_create(struct cpi_dev *cpi)
{
	struct plat_csi_pipeline *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	list_add_tail(&p->list, &cpi->pipelines);

	p->ep.ops = &plat_csi_pipeline_ops;
	return &p->ep;
}
static void plat_csi_pipelines_free(struct cpi_dev *cpi)
{
	while (!list_empty(&cpi->pipelines)) {
		struct plat_csi_pipeline *p;

		p = list_entry(cpi->pipelines.next, typeof(*p), list);
		list_del(&p->list);
		kfree(p);
	}
}

/* Parse port node and register as a sub-device any sensor specified there. */
static int plat_csi_parse_port_node(struct cpi_dev *plat_csi,
				    struct device_node *port,
				    unsigned int index)
{
	struct device_node *rem, *ep;
	struct v4l2_fwnode_endpoint endpoint = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct plat_csi_source_info *pd = &plat_csi->sensor[index].pdata;
	int ret;

	/* Assume here a port node can have only one endpoint node. */
	ep = of_get_next_child(port, NULL);
	if (!ep)
		return 0;

	endpoint.bus_type |= V4L2_MBUS_CSI2_DPHY;

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &endpoint);
	if (ret) {
		of_node_put(ep);
		return ret-1;
	}

	if (WARN_ON(endpoint.base.port == 0) || index >= PLAT_MAX_SENSORS) {
		of_node_put(ep);
		return -EINVAL;
	}

	pd->mux_id = (endpoint.base.port - 1) & 0x1;

	rem = of_graph_get_remote_port_parent(ep);
	of_node_put(ep);
	if (rem == NULL) {
		dev_info(plat_csi->dev, "Remote device at %s not found\n",
			 ep->full_name);
		return 0;
	}

	if (WARN_ON(index >= ARRAY_SIZE(plat_csi->sensor)))
		return -EINVAL;


	plat_csi->sensor[index].asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
	plat_csi->sensor[index].asd.match.fwnode = of_fwnode_handle(rem);
	plat_csi->async_subdevs[index] = &plat_csi->sensor[index].asd;

	plat_csi->num_sensors++;
	of_node_put(rem);

	return 0;
}

/* Register all SoC external sub-devices */
static int plat_csi_register_sensor_entities(struct cpi_dev *plat_csi)
{
	struct device_node *parent;
	struct device_node *node;
	struct device_node *port;
	int index = 0;
	int ret;

	plat_csi->num_sensors = 0;

	parent = of_get_next_parent(plat_csi->pdev->dev.of_node);

	/* Attach sensors linked to MIPI CSI-2 receivers */
	for_each_available_child_of_node(parent, node) {

		if (of_node_cmp(node->name, "csi2"))
			continue;

		/* The csi2 node can have only port subnode. */
		port = of_get_next_child(node, NULL);
		if (!port)
			continue;

		ret = plat_csi_parse_port_node(plat_csi, port, index);
		if (ret < 0)
			return ret;
		index++;
	}
	return 0;
}

static int register_csi_platform_entity(struct cpi_dev *plat_csi,
				     struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_subdev *sd;
	int ret = -EPROBE_DEFER;

	/* Lock to ensure dev->driver won't change. */
	device_lock(dev);
	if (!dev->driver || !try_module_get(dev->driver->owner))
		goto dev_unlock;

	if (WARN_ON(plat_csi->csi_sd))
		return -EBUSY;

	sd = dev_get_drvdata(dev);
	/* Some subdev didn't probe successfully id drvdata is NULL */
	if (sd) {
		sd->grp_id = GRP_ID_CSI;
		ret = v4l2_device_register_subdev(&plat_csi->v4l2_dev, sd);
		if(!ret)
			plat_csi->csi_sd = sd;
		else
			v4l2_err(&plat_csi->v4l2_dev,
				 "Failed to register MIPI-CSI (%d)\n", ret);
	} else
		dev_err(&plat_csi->pdev->dev, "%s no drvdata\n", dev_name(dev));

	module_put(dev->driver->owner);
dev_unlock:
	device_unlock(dev);
	if (ret == -EPROBE_DEFER)
		dev_info(&plat_csi->pdev->dev, "deferring %s device registration\n",
			dev_name(dev));
	else if (ret < 0)
		dev_err(&plat_csi->pdev->dev, "%s device registration failed (%d)\n",
			dev_name(dev), ret);
	return ret;
}

static int plat_csi_register_platform_entities(struct cpi_dev *plat_csi,
					       struct device_node *parent)
{
	struct device_node *node;
	struct platform_device *pdev;
	int ret = 0;

	parent = of_get_next_parent(parent);

	for_each_available_child_of_node(parent, node) {
		pdev = of_find_device_by_node(node);
		if (!pdev)
			continue;

		/* If driver of any entity isn't ready try all again later. */
		if (!strcmp(node->name, CSI_OF_NODE_NAME)) {
			ret = register_csi_platform_entity(plat_csi, pdev);
		}
		put_device(&pdev->dev);
		if (ret < 0)
			break;
	}

	return ret;
}

static void plat_csi_unregister_entities(struct cpi_dev *plat_csi)
{
	if (plat_csi->csi_sd != NULL){
		v4l2_device_unregister_subdev(plat_csi->csi_sd);
		plat_csi->csi_sd = NULL;
	}
	if (plat_csi->cpi_sd != NULL) {
		v4l2_device_unregister_subdev(plat_csi->cpi_sd);
		plat_csi->cpi_sd = NULL;
	}
	v4l2_info(&plat_csi->v4l2_dev, "Unregistered all entities\n");
}

static int plat_csi_create_links(struct cpi_dev *plat_csi)
{
	struct v4l2_subdev *csi_sensor[CSI_MAX_ENTITIES] = { NULL };
	struct v4l2_subdev *sensor;
	struct media_entity *source;
	struct media_entity *sink;
	struct plat_csi_source_info *pdata;
	int i, pad, ret = 0;

	for (i = 0; i < plat_csi->num_sensors; i++) {
		if (plat_csi->sensor[i].subdev == NULL)
			continue;

		sensor = plat_csi->sensor[i].subdev;
		pdata = v4l2_get_subdev_hostdata(sensor);
		if (!pdata)
			continue;

		source = &sensor->entity;
		sink = &plat_csi->csi_sd->entity;
		if (WARN(sink == NULL, "dw-mipi-csi module is not loaded!\n"))
			return -EINVAL;

		pad = sensor->entity.num_pads - 1;
		ret = media_create_pad_link(source, pad, sink, CSI_PAD_SINK,
				      MEDIA_LNK_FL_IMMUTABLE |
				      MEDIA_LNK_FL_ENABLED);
		if (ret)
			return ret;

		v4l2_info(&plat_csi->v4l2_dev, "created link [%s] -> [%s]\n",
			  source->name, sink->name);
		csi_sensor[pdata->mux_id] = sensor;
	}

	if ((plat_csi->cpi_sd == NULL) && (plat_csi->csi_sd == NULL)) {
		pr_info("no link\n");
	}

	source = &plat_csi->csi_sd->entity;
	sink = &plat_csi->cpi_sd->entity;
	ret = media_create_pad_link(source, CSI_PAD_SOURCE, sink,
				    CPI_PAD_SINK_CSI,
				    MEDIA_LNK_FL_ENABLED);
	if (ret)
		return ret;

	v4l2_info(&plat_csi->v4l2_dev, "created link [%s] -> [%s]\n",
	  source->name, sink->name);

	source = &plat_csi->cpi_sd->entity;
	sink = &plat_csi->ve.vdev.entity;
	ret = media_create_pad_link(source, CPI_PAD_SOURCE, sink,
				 VIDEO_DEV_PAD_SINK_CPI, MEDIA_LNK_FL_ENABLED);
	if (ret)
		return ret;
	v4l2_info(&plat_csi->v4l2_dev, "created link [%s] -> [%s]\n",
		  source->name, sink->name);

	return ret;
}

static int plat_csi_modify_pipeline(struct media_entity *entity, bool enable)
{
	struct plat_csi_video_entity *ve;
	struct plat_csi_pipeline *p;
	struct video_device *vdev;
	int ret;

	vdev = media_entity_to_video_device(entity);

	if (vdev->entity.use_count == 0)
		return 0;

	ve = vdev_to_plat_csi_video_entity(vdev);
	p = to_plat_csi_pipeline(ve->pipe);

	if (enable)
		ret = plat_csi_pipeline_open(ve->pipe, entity, true);
	else
		ret = plat_csi_pipeline_close(ve->pipe);

	if (ret == 0 && !enable)
		memset(p->subdevs, 0, sizeof(p->subdevs));

	return ret;
}

/* Locking: called with entity->parent->graph_mutex mutex held. */
static int plat_csi_modify_pipelines(struct media_entity *entity, bool enable,
				     struct media_graph *graph)
{
	struct media_entity *entity_err = entity;
	int ret;

	/*
	 * Walk current graph and call the pipeline open/close routine for each
	 * opened video node that belongs to the graph of entities connected
	 * through active links. This is needed as we cannot power on/off the
	 * subdevs in random order.
	 */
	media_graph_walk_start(graph, entity);

	while ((entity = media_graph_walk_next(graph))) {
		if (!is_media_entity_v4l2_video_device(entity))
			continue;

		ret  = plat_csi_modify_pipeline(entity, enable);

		if (ret < 0)
			goto err;
	}

	return 0;
 err:
	media_graph_walk_start(graph, entity_err);

	while ((entity_err = media_graph_walk_next(graph))) {
		if (!is_media_entity_v4l2_video_device(entity_err))
			continue;

		plat_csi_modify_pipeline(entity_err, !enable);

		if (entity_err == entity)
			break;
	}

	return ret;
}

static int plat_csi_link_notify(struct media_link *link,
				unsigned int flags,
				unsigned int notification)
{
	struct media_graph *graph;
	struct media_entity *sink = link->sink->entity;
	int ret = 0;

	graph = &container_of(link->graph_obj.mdev, struct cpi_dev,
		media_dev)->link_setup_graph;
	/* Before link disconnection */
	if (notification == MEDIA_DEV_NOTIFY_PRE_LINK_CH) {
		ret = media_graph_walk_init(graph, link->graph_obj.mdev);
		if (ret)
			return ret;
		if (!(flags & MEDIA_LNK_FL_ENABLED))
			ret = plat_csi_modify_pipelines(sink, false, graph);
		/* After link activation */
	} else if (notification == MEDIA_DEV_NOTIFY_POST_LINK_CH) {
		if (link->flags & MEDIA_LNK_FL_ENABLED)
			ret = plat_csi_modify_pipelines(sink, true, graph);
		media_graph_walk_cleanup(graph);
	}

	return ret ? -EPIPE : 0;
}

static const struct media_device_ops cpi_media_ops = {
	.link_notify = plat_csi_link_notify,
};

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_subdev *asd)
{
	struct cpi_dev *plat_csi = notifier_to_plat_csi(notifier);
	struct plat_csi_sensor_info *si = NULL;
	int i;

	/* Find platform data for this sensor subdev */
	for (i = 0; i < ARRAY_SIZE(plat_csi->sensor); i++)
		if (plat_csi->sensor[i].asd.match.fwnode ==
			of_fwnode_handle(subdev->dev->of_node))
			si = &plat_csi->sensor[i];

	if (si == NULL)
		return -EINVAL;

	v4l2_set_subdev_hostdata(subdev, &si->pdata);
	plat_csi->num_sensors++;
	subdev->grp_id = GRP_ID_SENSOR + (plat_csi->num_sensors - 1);
	si->subdev = subdev;

	v4l2_info(&plat_csi->v4l2_dev, "Registered sensor subdevice: %s (%d)\n",
			  subdev->name, plat_csi->num_sensors);

	return 0;
}

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct cpi_dev *cpi = notifier_to_plat_csi(notifier);
	int ret;

	mutex_lock(&cpi->media_dev.graph_mutex);

	ret = plat_csi_create_links(cpi);
	if (ret < 0)
		goto unlock;

	ret = v4l2_device_register_subdev_nodes(&cpi->v4l2_dev);
unlock:
	mutex_unlock(&cpi->media_dev.graph_mutex);
	return ret;
}
static const struct v4l2_async_notifier_operations dw_csi_async_ops = {
	.bound = subdev_notifier_bound,
	.complete = subdev_notifier_complete,
};

int plat_csi_probe(struct platform_device *pdev, struct cpi_dev *cpi)
{
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	struct plat_csi_media_pipeline *ep;
	int ret, i;

	spin_lock_init(&cpi->slock);
	INIT_LIST_HEAD(&cpi->pipelines);

	strlcpy(cpi->media_dev.model, "Alif Platform",
		sizeof(cpi->media_dev.model));
	cpi->media_dev.ops = &cpi_media_ops;
	cpi->media_dev.dev = dev;

	v4l2_dev = &cpi->v4l2_dev;
	v4l2_dev->mdev = &cpi->media_dev;
	strlcpy(v4l2_dev->name, "alif-cpi", sizeof(v4l2_dev->name));

	media_device_init(&cpi->media_dev);

	ret = v4l2_device_register(dev, &cpi->v4l2_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register v4l2_device: %d\n", ret);
		return ret;
	}

	v4l2_async_notifier_init(&cpi->subdev_notifier);

	ret = plat_csi_register_platform_entities(cpi, dev->of_node);
	if (ret)
		goto err_m_ent;

	ep = plat_csi_pipeline_create(cpi);
	if (!ep)
		return -ENOMEM;

	v4l2_set_subdev_hostdata(&cpi->subdev, ep);

	cpi->subdev.grp_id = GRP_ID_CPI;
	ret = v4l2_device_register_subdev(&cpi->v4l2_dev, &cpi->subdev);

	if (!ret)
		cpi->cpi_sd = &cpi->subdev;
	else
		v4l2_err(&cpi->v4l2_dev,
			 "Failed to register CPI (%d)\n", ret);

	ret = plat_csi_register_sensor_entities(cpi);
	if (ret)
		goto err_m_ent;

	if (cpi->num_sensors > 0) {
		cpi->subdev_notifier.ops = &dw_csi_async_ops;
		cpi->num_sensors = 0;

		for(i = 0; i < PLAT_MAX_SENSORS; i++){
			if(cpi->async_subdevs[i]) {
				ret = v4l2_async_notifier_add_subdev(&cpi->subdev_notifier, cpi->async_subdevs[i]);
				if (ret) {
					return ret;
				}
			}
		}

		ret = v4l2_async_notifier_register(&cpi->v4l2_dev,
						   &cpi->subdev_notifier);
		if (ret)
			goto err_m_ent;
	}
	return 0;

err_m_ent:
	plat_csi_unregister_entities(cpi);
	media_device_unregister(&cpi->media_dev);
	media_device_cleanup(&cpi->media_dev);
	v4l2_device_unregister(&cpi->v4l2_dev);
	return ret;
}

int plat_csi_remove(struct platform_device *pdev)
{
	struct cpi_dev *dev = platform_get_drvdata(pdev);

	if (!dev)
		return 0;

	v4l2_async_notifier_unregister(&dev->subdev_notifier);

	v4l2_device_unregister(&dev->v4l2_dev);
	plat_csi_unregister_entities(dev);
	plat_csi_pipelines_free(dev);
	media_device_unregister(&dev->media_dev);

	dev_info(&pdev->dev, "Driver removed\n");

	return 0;
}

