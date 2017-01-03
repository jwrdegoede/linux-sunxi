/*
 * sha256_base.h - core logic for SHA-256 implementations
 *
 * Copyright (C) 2015 Linaro Ltd <ard.biesheuvel@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <crypto/internal/hash.h>
#include <crypto/sha.h>
#include <linux/crypto.h>
#include <linux/module.h>

#include <asm/unaligned.h>

typedef void (sha256_block_fn)(struct sha256_state *sst, u8 const *src,
			       int blocks);

static inline void sha224_init_direct(struct sha256_state *sctx)
{
	sctx->state[0] = SHA224_H0;
	sctx->state[1] = SHA224_H1;
	sctx->state[2] = SHA224_H2;
	sctx->state[3] = SHA224_H3;
	sctx->state[4] = SHA224_H4;
	sctx->state[5] = SHA224_H5;
	sctx->state[6] = SHA224_H6;
	sctx->state[7] = SHA224_H7;
	sctx->count = 0;
}

static inline int sha224_base_init(struct shash_desc *desc)
{
	sha224_init_direct(shash_desc_ctx(desc));
	return 0;
}

static inline int sha256_base_init(struct shash_desc *desc)
{
	sha256_init_direct(shash_desc_ctx(desc));
	return 0;
}

static inline void __sha256_base_do_update(struct sha256_state *sctx,
					   const u8 *data,
					   unsigned int len,
					   sha256_block_fn *block_fn)
{
	unsigned int partial = sctx->count % SHA256_BLOCK_SIZE;

	sctx->count += len;

	if (unlikely((partial + len) >= SHA256_BLOCK_SIZE)) {
		int blocks;

		if (partial) {
			int p = SHA256_BLOCK_SIZE - partial;

			memcpy(sctx->buf + partial, data, p);
			data += p;
			len -= p;

			block_fn(sctx, sctx->buf, 1);
		}

		blocks = len / SHA256_BLOCK_SIZE;
		len %= SHA256_BLOCK_SIZE;

		if (blocks) {
			block_fn(sctx, data, blocks);
			data += blocks * SHA256_BLOCK_SIZE;
		}
		partial = 0;
	}
	if (len)
		memcpy(sctx->buf + partial, data, len);
}

static inline int sha256_base_do_update(struct shash_desc *desc,
					const u8 *data,
					unsigned int len,
					sha256_block_fn *block_fn)
{
	__sha256_base_do_update(shash_desc_ctx(desc), data, len, block_fn);
	return 0;
}

static inline void sha256_do_finalize_direct(struct sha256_state *sctx,
					     sha256_block_fn *block_fn)
{
	const int bit_offset = SHA256_BLOCK_SIZE - sizeof(__be64);
	__be64 *bits = (__be64 *)(sctx->buf + bit_offset);
	unsigned int partial = sctx->count % SHA256_BLOCK_SIZE;

	sctx->buf[partial++] = 0x80;
	if (partial > bit_offset) {
		memset(sctx->buf + partial, 0x0, SHA256_BLOCK_SIZE - partial);
		partial = 0;

		block_fn(sctx, sctx->buf, 1);
	}

	memset(sctx->buf + partial, 0x0, bit_offset - partial);
	*bits = cpu_to_be64(sctx->count << 3);
	block_fn(sctx, sctx->buf, 1);
}

static inline int sha256_base_do_finalize(struct shash_desc *desc,
					  sha256_block_fn *block_fn)
{
	sha256_do_finalize_direct(shash_desc_ctx(desc), block_fn);
	return 0;
}

static inline void __sha256_base_finish(struct sha256_state *sctx,
					unsigned int digest_size, u8 *out)
{
	__be32 *digest = (__be32 *)out;
	int i;

	for (i = 0; digest_size > 0; i++, digest_size -= sizeof(__be32))
		put_unaligned_be32(sctx->state[i], digest++);

	*sctx = (struct sha256_state){};
}

static inline int sha256_base_finish(struct shash_desc *desc, u8 *out)
{
	__sha256_base_finish(shash_desc_ctx(desc),
			     crypto_shash_digestsize(desc->tfm), out);
	return 0;
}
