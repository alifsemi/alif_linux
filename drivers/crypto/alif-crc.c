// SPDX-License-Identifier: GPL-2.0-only

/*
 * Alif CRC Driver
 * Copyright (2022) Alif Semiconductor
 */

#include <linux/bitrev.h>
#include <linux/crc32.h>
#include <linux/crc32poly.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/byteorder.h>
#include <crypto/internal/hash.h>
#include <asm/unaligned.h>

#define DRIVER_NAME             "alif-crc32"
#define CHKSUM_DIGEST_SIZE      4
#define CHKSUM_BLOCK_SIZE       1

/* Registers */
#define CRC_CTRL                0x00000000
#define CRC_SEED                0x00000010
#define CRC_RESULT              0x00000018
#define MAX_LEN                 156000
#define CRC_DATA                0x00000060
#define CRC_DATA16              0x00000020

#define CRC_INIT_VALUE_CRC32          0xd25
#define CRC_INIT_VALUE_CRC32C         0xd2d
#define CRC_INIT_VALUE_CRC16          0x13
#define CRC_INIT_VALUE_CRC16_CCITT    0x1b
#define CRC_INIT_VALUE_CRC8           0x1

#define CRC32_POLY_CRC32        1
#define CRC32_POLY_CRC32C       2
#define CRC16_POLY_CRC16        3
#define CRC16_POLY_CRC16_CCITT  4
#define CRC8_POLY_CRC8          5

struct alif_crc {
	struct device    *dev;
	/* lock to protect access to the hardware */
	spinlock_t lock;
};

struct alif_crc_ctx {
	u32 key;
	u32 poly;
	u32 init;
};

struct alif_crc_desc_ctx {
	u32 partial_result;
	struct alif_crc *crc;
};

struct alif_crc *crc_init;
void __iomem *regs;

static int crc32_cra_init(struct crypto_tfm *tfm)
{
	struct alif_crc_ctx *mtx = crypto_tfm_ctx(tfm);

	mtx->key = 0xffffffff;
	mtx->poly = CRC32_POLY_CRC32;
	mtx->init = CRC_INIT_VALUE_CRC32;

	return 0;
}

static int crc32c_cra_init(struct crypto_tfm *tfm)
{
	struct alif_crc_ctx *mtx = crypto_tfm_ctx(tfm);

	mtx->key = 0xffffffff;
	mtx->poly = CRC32_POLY_CRC32C;
	mtx->init = CRC_INIT_VALUE_CRC32C;

	return 0;
}

static int crc16_cra_init(struct crypto_tfm *tfm)
{
	struct alif_crc_ctx *mtx = crypto_tfm_ctx(tfm);

	mtx->key = 0x0;
	mtx->poly = CRC16_POLY_CRC16;
	mtx->init = CRC_INIT_VALUE_CRC16;

	return 0;
}

static int crc16_ccitt_cra_init(struct crypto_tfm *tfm)
{
	struct alif_crc_ctx *mtx = crypto_tfm_ctx(tfm);

	mtx->key = 0x0;
	mtx->poly = CRC16_POLY_CRC16_CCITT;
	mtx->init = CRC_INIT_VALUE_CRC16_CCITT;

	return 0;
}

static int crc8_init(struct crypto_tfm *tfm)
{
	struct alif_crc_ctx *mtx = crypto_tfm_ctx(tfm);

	mtx->key = 0x0;
	mtx->poly = CRC8_POLY_CRC8;
	mtx->init = CRC_INIT_VALUE_CRC8;

	return 0;
}

static int crc_setkey(struct crypto_shash *tfm, const u8 *key,
			    unsigned int keylen)
{
	struct alif_crc_ctx *mctx = crypto_shash_ctx(tfm);

	if (keylen != sizeof(u32)) {
		crypto_shash_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	mctx->key = get_unaligned_le32(key);

	return 0;
}

static int crc_init_crc_ctx(struct shash_desc *desc)
{
	struct alif_crc_desc_ctx *ctx = shash_desc_ctx(desc);
	struct alif_crc_ctx *mctx = crypto_shash_ctx(desc->tfm);
	struct alif_crc *crc = ctx->crc;
	u32 key_value;

	ctx->crc = crc_init;

	key_value = mctx->key;


	spin_lock(&crc->lock);

	/* set the seed */
	writel(mctx->key, regs + CRC_SEED);

	/* set alg, and load */
	writel(mctx->init, regs + CRC_CTRL);

	ctx->partial_result = readl(regs + CRC_RESULT);

	spin_unlock(&crc->lock);

	return 0;
}

static unsigned int crc32_update_unaligned(u32 crc, const u8 *data, unsigned int length)
{
	return crc32_le(crc, data, length) ^ (~0);
}

static unsigned int crc32c_update_unaligned(u32 crc, const u8 *data, unsigned int length)
{
	return __crc32c_le(crc, data, length) ^ (~0);
}

static int crc_update(struct shash_desc *desc, const u8 *datain,
			    unsigned int length)
{
	struct alif_crc_desc_ctx *ctx = shash_desc_ctx(desc);
	struct alif_crc_ctx *mctx = crypto_shash_ctx(desc->tfm);
	struct alif_crc *crc = ctx->crc;
	unsigned int i;
	u32 value;
	u32 num_writes;
	u32 *d32;
	u32 result;
	u32 extra_bytes;

	if (datain == NULL) {
		return 0;
	}

	num_writes = length / sizeof(u32);
	extra_bytes = length % sizeof(u32);

	d32 = (u32 *)datain;

	spin_lock(&crc->lock);

	for (i = 0; i < num_writes; i++) {
		value = *(d32++);
		value = __be32_to_cpu(value);
		writel(value, regs+CRC_DATA);
	}

	ctx->partial_result = readl(regs + CRC_RESULT);

	spin_unlock(&crc->lock);

	if (extra_bytes) {
		if (mctx->poly == CRC32_POLY_CRC32) {
			result = ctx->partial_result ^ (~0);
			ctx->partial_result = crc32_update_unaligned(result, datain + (num_writes * sizeof(u32)), extra_bytes);
		} else {
			result = ctx->partial_result ^ (~0);
			ctx->partial_result = crc32c_update_unaligned(result, datain + (num_writes * sizeof(u32)), extra_bytes);
		}
	}

	return 0;
}

static int crc16_update(struct shash_desc *desc, const u8 *datain,
			    unsigned int length)
{
	struct alif_crc_desc_ctx *ctx = shash_desc_ctx(desc);
	struct alif_crc *crc = ctx->crc;
	unsigned int i;
	u8 value;

	if (datain == NULL) {
		return 0;
	}

	spin_lock(&crc->lock);

	for (i = 0; i < length; i++) {
		value = *(datain++);
		writeb(value, regs+CRC_DATA16);
	}

	ctx->partial_result = readl(regs + CRC_RESULT);

	spin_unlock(&crc->lock);

	return 0;
}

static int crc_final(struct shash_desc *desc, u8 *out)
{
	struct alif_crc_desc_ctx *ctx = shash_desc_ctx(desc);
	struct alif_crc_ctx *mctx = crypto_shash_ctx(desc->tfm);

	if (mctx->poly == CRC32_POLY_CRC32 || mctx->poly == CRC32_POLY_CRC32C) {
		put_unaligned_le32(ctx->partial_result, out);
	} else if (mctx->poly == CRC16_POLY_CRC16 || mctx->poly == CRC16_POLY_CRC16_CCITT) { /* 16 bit crc */
		put_unaligned_le16(ctx->partial_result, out);
	} else { /* 8 bit */
		*out = (u8)ctx->partial_result;
	}

	return 0;
}

static int crc_finup(struct shash_desc *desc, const u8 *data,
			   unsigned int length, u8 *out)
{
	return crc_update(desc, data, length) ?:
	       crc_final(desc, out);
}

static int crc_digest(struct shash_desc *desc, const u8 *data,
			    unsigned int length, u8 *out)
{
	return crc_init_crc_ctx(desc) ?: crc_finup(desc, data, length, out);
}

static struct shash_alg crc_alg[] = {
	{
	.setkey         = crc_setkey,
	.init           = crc_init_crc_ctx,
	.update         = crc_update,
	.final          = crc_final,
	.finup          = crc_finup,
	.digest         = crc_digest,
	.descsize       = sizeof(struct alif_crc_desc_ctx),
	.digestsize     = CHKSUM_DIGEST_SIZE,
	.base           = {
		.cra_name               = "alif-crc",
		.cra_driver_name        = DRIVER_NAME,
		.cra_priority           = 200,
		.cra_flags		= CRYPTO_ALG_OPTIONAL_KEY,
		.cra_blocksize          = CHKSUM_BLOCK_SIZE,
		.cra_alignmask          = 3,
		.cra_ctxsize            = sizeof(struct alif_crc_ctx),
		.cra_module             = THIS_MODULE,
		.cra_init               = crc32_cra_init,
	}
	},
	{
	.setkey         = crc_setkey,
	.init           = crc_init_crc_ctx,
	.update         = crc_update,
	.final          = crc_final,
	.finup          = crc_finup,
	.digest         = crc_digest,
	.descsize       = sizeof(struct alif_crc_desc_ctx),
	.digestsize     = CHKSUM_DIGEST_SIZE,
	.base           = {
		.cra_name               = "alif-crcc",
		.cra_driver_name        = DRIVER_NAME,
		.cra_priority           = 200,
		.cra_flags		= CRYPTO_ALG_OPTIONAL_KEY,
		.cra_blocksize          = CHKSUM_BLOCK_SIZE,
		.cra_alignmask          = 3,
		.cra_ctxsize            = sizeof(struct alif_crc_ctx),
		.cra_module             = THIS_MODULE,
		.cra_init               = crc32c_cra_init,
	}
	},
	{
	.setkey         = crc_setkey,
	.init           = crc_init_crc_ctx,
	.update         = crc16_update,
	.final          = crc_final,
	.finup          = crc_finup,
	.digest         = crc_digest,
	.descsize       = sizeof(struct alif_crc_desc_ctx),
	.digestsize     = CHKSUM_DIGEST_SIZE,
	.base           = {
		.cra_name               = "alif-crc16",
		.cra_driver_name        = DRIVER_NAME,
		.cra_priority           = 200,
		.cra_flags		= CRYPTO_ALG_OPTIONAL_KEY,
		.cra_blocksize          = CHKSUM_BLOCK_SIZE,
		.cra_alignmask          = 3,
		.cra_ctxsize            = sizeof(struct alif_crc_ctx),
		.cra_module             = THIS_MODULE,
		.cra_init               = crc16_cra_init,
	}
	},
	{
	.setkey         = crc_setkey,
	.init           = crc_init_crc_ctx,
	.update         = crc16_update,
	.final          = crc_final,
	.finup          = crc_finup,
	.digest         = crc_digest,
	.descsize       = sizeof(struct alif_crc_desc_ctx),
	.digestsize     = CHKSUM_DIGEST_SIZE,
	.base           = {
		.cra_name               = "alif-crc16-ccitt",
		.cra_driver_name        = DRIVER_NAME,
		.cra_priority           = 200,
		.cra_flags		= CRYPTO_ALG_OPTIONAL_KEY,
		.cra_blocksize          = CHKSUM_BLOCK_SIZE,
		.cra_alignmask          = 3,
		.cra_ctxsize            = sizeof(struct alif_crc_ctx),
		.cra_module             = THIS_MODULE,
		.cra_init               = crc16_ccitt_cra_init,
	}
	},
	{
	.setkey         = crc_setkey,
	.init           = crc_init_crc_ctx,
	/* The update function is same for crc8 and 16 */
	.update         = crc16_update,
	.final          = crc_final,
	.finup          = crc_finup,
	.digest         = crc_digest,
	.descsize       = sizeof(struct alif_crc_desc_ctx),
	.digestsize     = CHKSUM_DIGEST_SIZE,
	.base           = {
		.cra_name               = "alif-crc8",
		.cra_driver_name        = DRIVER_NAME,
		.cra_priority           = 200,
		.cra_flags		= CRYPTO_ALG_OPTIONAL_KEY,
		.cra_blocksize          = CHKSUM_BLOCK_SIZE,
		.cra_alignmask          = 3,
		.cra_ctxsize            = sizeof(struct alif_crc_ctx),
		.cra_module             = THIS_MODULE,
		.cra_init               = crc8_init,
	}
	}
};

static int alif_crc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct alif_crc *crc;
	struct resource *res;
	int ret;

	crc = devm_kzalloc(dev, sizeof(*crc), GFP_KERNEL);
	if (!crc)
		return -ENOMEM;

	crc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	regs = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(regs)) {
		dev_err(dev, "Cannot map CRC Registers\n");
		return PTR_ERR(regs);
	}

	platform_set_drvdata(pdev, crc);

	spin_lock_init(&crc->lock);

	crc_init = crc;

	ret = crypto_register_shashes(crc_alg, sizeof(crc_alg)/sizeof(struct shash_alg));
	if (ret) {
		dev_err(dev, "Failed to register\n");
		return ret;
	}

	return 0;
}

static int alif_crc_remove(struct platform_device *pdev)
{
	crypto_unregister_shashes(crc_alg, 2);
	return 0;
}

static const struct of_device_id alif_ids[] = {
	{.compatible = "alif,alif-crccode",},
	{},
};
MODULE_DEVICE_TABLE(of, alif_ids);

static struct platform_driver alif_crc_driver = {
	.probe  = alif_crc_probe,
	.remove = alif_crc_remove,
	.driver = {
		.name           = DRIVER_NAME,
		.of_match_table = alif_ids,
	},
};

module_platform_driver(alif_crc_driver);

MODULE_DESCRIPTION("Alif CRC hardware driver");

MODULE_LICENSE("GPL");
