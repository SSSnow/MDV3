/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>

#include "py/mpstate.h"
#include "py/qstr.h"
#include "py/gc.h"

// NOTE: we are using linear arrays to store and search for qstr's (unique strings, interned strings)
// ultimately we will replace this with a static hash table of some kind
// also probably need to include the length in the string data, to allow null bytes in the string

#if 0 // print debugging info
#define DEBUG_printf DEBUG_printf
#else // don't print debugging info
#define DEBUG_printf(...) (void)0
#endif

// A qstr is an index into the qstr pool.
// The data for a qstr contains (hash, length, data):
//  - hash (configurable number of bytes)
//  - length (configurable number of bytes)
//  - data ("length" number of bytes)
//  - \0 terminated (so they can be printed using printf)

#if MICROPY_QSTR_BYTES_IN_HASH == 1
    #define Q_HASH_MASK (0xff)
    #define Q_GET_HASH(q) ((mp_uint_t)(q)[0])
    #define Q_SET_HASH(q, hash) do { (q)[0] = (hash); } while (0)
#elif MICROPY_QSTR_BYTES_IN_HASH == 2
    #define Q_HASH_MASK (0xffff)
    #define Q_GET_HASH(q) ((mp_uint_t)(q)[0] | ((mp_uint_t)(q)[1] << 8))
    #define Q_SET_HASH(q, hash) do { (q)[0] = (hash); (q)[1] = (hash) >> 8; } while (0)
#else
    #error unimplemented qstr hash decoding
#endif
#define Q_GET_ALLOC(q)  (MICROPY_QSTR_BYTES_IN_HASH + MICROPY_QSTR_BYTES_IN_LEN + Q_GET_LENGTH(q) + 1)
#define Q_GET_DATA(q)   ((q) + MICROPY_QSTR_BYTES_IN_HASH + MICROPY_QSTR_BYTES_IN_LEN)
#if MICROPY_QSTR_BYTES_IN_LEN == 1
    #define Q_GET_LENGTH(q) ((q)[MICROPY_QSTR_BYTES_IN_HASH])
    #define Q_SET_LENGTH(q, len) do { (q)[MICROPY_QSTR_BYTES_IN_HASH] = (len); } while (0)
#elif MICROPY_QSTR_BYTES_IN_LEN == 2
    #define Q_GET_LENGTH(q) ((q)[MICROPY_QSTR_BYTES_IN_HASH] | ((q)[MICROPY_QSTR_BYTES_IN_HASH + 1] << 8))
    #define Q_SET_LENGTH(q, len) do { (q)[MICROPY_QSTR_BYTES_IN_HASH] = (len); (q)[MICROPY_QSTR_BYTES_IN_HASH + 1] = (len) >> 8; } while (0)
#else
    #error unimplemented qstr length decoding
#endif

// this must match the equivalent function in makeqstrdata.py
mp_uint_t qstr_compute_hash(const byte *data, size_t len) {
    // djb2 algorithm; see http://www.cse.yorku.ca/~oz/hash.html
    mp_uint_t hash = 5381;
    for (const byte *top = data + len; data < top; data++) {
        hash = ((hash << 5) + hash) ^ (*data); // hash * 33 ^ data
    }
    hash &= Q_HASH_MASK;
    // Make sure that valid hash is never zero, zero means "hash not computed"
    if (hash == 0) {
        hash++;
    }
    return hash;
}

const qstr_pool_t mp_qstr_const_pool = {
    NULL,               // no previous pool
    0,                  // no previous pool
    10,                 // set so that the first dynamically allocated pool is twice this size; must be <= the len (just below)
    MP_QSTRnumber_of,   // corresponds to number of strings in array just below
    {
#ifndef __QSTR_EXTRACT
#define QDEF(id, str) str,
#include "genhdr/qstrdefs.generated.h"
#undef QDEF
#endif
    },
};

#ifdef MICROPY_QSTR_EXTRA_POOL
extern const qstr_pool_t MICROPY_QSTR_EXTRA_POOL;
#define CONST_POOL MICROPY_QSTR_EXTRA_POOL
#else
#define CONST_POOL mp_qstr_const_pool
#endif

void qstr_init(void) {
    MP_STATE_VM(last_pool) = (qstr_pool_t*)&CONST_POOL; // we won't modify the const_pool since it has no allocated room left
    MP_STATE_VM(qstr_last_chunk) = NULL;
}

STATIC const byte *find_qstr(qstr q) {
    // search pool for this qstr
    for (qstr_pool_t *pool = MP_STATE_VM(last_pool); pool != NULL; pool = pool->prev) {
        if (q >= pool->total_prev_len) {
            return pool->qstrs[q - pool->total_prev_len];
        }
    }

    // not found
    return 0;
}

STATIC qstr qstr_add(const byte *q_ptr) {
    DEBUG_printf("QSTR: add hash=%d len=%d data=%.*s\n", Q_GET_HASH(q_ptr), Q_GET_LENGTH(q_ptr), Q_GET_LENGTH(q_ptr), Q_GET_DATA(q_ptr));

    // make sure we have room in the pool for a new qstr
    if (MP_STATE_VM(last_pool)->len >= MP_STATE_VM(last_pool)->alloc) {
        qstr_pool_t *pool = m_new_obj_var(qstr_pool_t, const char*, MP_STATE_VM(last_pool)->alloc * 2);
        pool->prev = MP_STATE_VM(last_pool);
        pool->total_prev_len = MP_STATE_VM(last_pool)->total_prev_len + MP_STATE_VM(last_pool)->len;
        pool->alloc = MP_STATE_VM(last_pool)->alloc * 2;
        pool->len = 0;
        MP_STATE_VM(last_pool) = pool;
        DEBUG_printf("QSTR: allocate new pool of size %d\n", MP_STATE_VM(last_pool)->alloc);
    }

    // add the new qstr
    MP_STATE_VM(last_pool)->qstrs[MP_STATE_VM(last_pool)->len++] = q_ptr;

    // return id for the newly-added qstr
    return MP_STATE_VM(last_pool)->total_prev_len + MP_STATE_VM(last_pool)->len - 1;
}

qstr qstr_find_strn(const char *str, size_t str_len) {
    // work out hash of str
    mp_uint_t str_hash = qstr_compute_hash((const byte*)str, str_len);

    // search pools for the data
    for (qstr_pool_t *pool = MP_STATE_VM(last_pool); pool != NULL; pool = pool->prev) {
        for (const byte **q = pool->qstrs, **q_top = pool->qstrs + pool->len; q < q_top; q++) {
            if (Q_GET_HASH(*q) == str_hash && Q_GET_LENGTH(*q) == str_len && memcmp(Q_GET_DATA(*q), str, str_len) == 0) {
                return pool->total_prev_len + (q - pool->qstrs);
            }
        }
    }

    // not found; return null qstr
    return 0;
}

qstr qstr_from_str(const char *str) {
    return qstr_from_strn(str, strlen(str));
}

qstr qstr_from_strn(const char *str, size_t len) {
    assert(len < (1 << (8 * MICROPY_QSTR_BYTES_IN_LEN)));
    qstr q = qstr_find_strn(str, len);
    if (q == 0) {
        // qstr does not exist in interned pool so need to add it

        // compute number of bytes needed to intern this string
        size_t n_bytes = MICROPY_QSTR_BYTES_IN_HASH + MICROPY_QSTR_BYTES_IN_LEN + len + 1;

        if (MP_STATE_VM(qstr_last_chunk) != NULL && MP_STATE_VM(qstr_last_used) + n_bytes > MP_STATE_VM(qstr_last_alloc)) {
            // not enough room at end of previously interned string so try to grow
            byte *new_p = m_renew_maybe(byte, MP_STATE_VM(qstr_last_chunk), MP_STATE_VM(qstr_last_alloc), MP_STATE_VM(qstr_last_alloc) + n_bytes, false);
            if (new_p == NULL) {
                // could not grow existing memory; shrink it to fit previous
                (void)m_renew(byte, MP_STATE_VM(qstr_last_chunk), MP_STATE_VM(qstr_last_alloc), MP_STATE_VM(qstr_last_used));
                MP_STATE_VM(qstr_last_chunk) = NULL;
            } else {
                // could grow existing memory
                MP_STATE_VM(qstr_last_alloc) += n_bytes;
            }
        }

        if (MP_STATE_VM(qstr_last_chunk) == NULL) {
            // no existing memory for the interned string so allocate a new chunk
            size_t al = n_bytes;
            if (al < MICROPY_ALLOC_QSTR_CHUNK_INIT) {
                al = MICROPY_ALLOC_QSTR_CHUNK_INIT;
            }
            MP_STATE_VM(qstr_last_chunk) = m_new_maybe(byte, al);
            if (MP_STATE_VM(qstr_last_chunk) == NULL) {
                // failed to allocate a large chunk so try with exact size
                MP_STATE_VM(qstr_last_chunk) = m_new(byte, n_bytes);
                al = n_bytes;
            }
            MP_STATE_VM(qstr_last_alloc) = al;
            MP_STATE_VM(qstr_last_used) = 0;
        }

        // allocate memory from the chunk for this new interned string's data
        byte *q_ptr = MP_STATE_VM(qstr_last_chunk) + MP_STATE_VM(qstr_last_used);
        MP_STATE_VM(qstr_last_used) += n_bytes;

        // store the interned strings' data
        mp_uint_t hash = qstr_compute_hash((const byte*)str, len);
        Q_SET_HASH(q_ptr, hash);
        Q_SET_LENGTH(q_ptr, len);
        memcpy(q_ptr + MICROPY_QSTR_BYTES_IN_HASH + MICROPY_QSTR_BYTES_IN_LEN, str, len);
        q_ptr[MICROPY_QSTR_BYTES_IN_HASH + MICROPY_QSTR_BYTES_IN_LEN + len] = '\0';
        q = qstr_add(q_ptr);
    }
    return q;
}

byte *qstr_build_start(size_t len, byte **q_ptr) {
    assert(len < (1 << (8 * MICROPY_QSTR_BYTES_IN_LEN)));
    *q_ptr = m_new(byte, MICROPY_QSTR_BYTES_IN_HASH + MICROPY_QSTR_BYTES_IN_LEN + len + 1);
    Q_SET_LENGTH(*q_ptr, len);
    return Q_GET_DATA(*q_ptr);
}

qstr qstr_build_end(byte *q_ptr) {
    qstr q = qstr_find_strn((const char*)Q_GET_DATA(q_ptr), Q_GET_LENGTH(q_ptr));
    if (q == 0) {
        size_t len = Q_GET_LENGTH(q_ptr);
        mp_uint_t hash = qstr_compute_hash(Q_GET_DATA(q_ptr), len);
        Q_SET_HASH(q_ptr, hash);
        q_ptr[MICROPY_QSTR_BYTES_IN_HASH + MICROPY_QSTR_BYTES_IN_LEN + len] = '\0';
        q = qstr_add(q_ptr);
    } else {
        m_del(byte, q_ptr, Q_GET_ALLOC(q_ptr));
    }
    return q;
}

mp_uint_t qstr_hash(qstr q) {
    return Q_GET_HASH(find_qstr(q));
}

size_t qstr_len(qstr q) {
    const byte *qd = find_qstr(q);
    return Q_GET_LENGTH(qd);
}

// XXX to remove!
const char *qstr_str(qstr q) {
    const byte *qd = find_qstr(q);
    return (const char*)Q_GET_DATA(qd);
}

const byte *qstr_data(qstr q, size_t *len) {
    const byte *qd = find_qstr(q);
    *len = Q_GET_LENGTH(qd);
    return Q_GET_DATA(qd);
}

void qstr_pool_info(size_t *n_pool, size_t *n_qstr, size_t *n_str_data_bytes, size_t *n_total_bytes) {
    *n_pool = 0;
    *n_qstr = 0;
    *n_str_data_bytes = 0;
    *n_total_bytes = 0;
    for (qstr_pool_t *pool = MP_STATE_VM(last_pool); pool != NULL && pool != &CONST_POOL; pool = pool->prev) {
        *n_pool += 1;
        *n_qstr += pool->len;
        for (const byte **q = pool->qstrs, **q_top = pool->qstrs + pool->len; q < q_top; q++) {
            *n_str_data_bytes += Q_GET_ALLOC(*q);
        }
        #if MICROPY_ENABLE_GC
        *n_total_bytes += gc_nbytes(pool); // this counts actual bytes used in heap
        #else
        *n_total_bytes += sizeof(qstr_pool_t) + sizeof(qstr) * pool->alloc;
        #endif
    }
    *n_total_bytes += *n_str_data_bytes;
}

#if MICROPY_PY_MICROPYTHON_MEM_INFO
void qstr_dump_data(void) {
    for (qstr_pool_t *pool = MP_STATE_VM(last_pool); pool != NULL && pool != &CONST_POOL; pool = pool->prev) {
        for (const byte **q = pool->qstrs, **q_top = pool->qstrs + pool->len; q < q_top; q++) {
            mp_printf(&mp_plat_print, "Q(%s)\n", Q_GET_DATA(*q));
        }
    }
}
#endif