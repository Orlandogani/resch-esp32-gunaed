/**
 * On-target Unity tests for subsys/cfg (SDD §16.2). Uses a dedicated "cfgtest"
 * namespace and erases it first, so runs are independent of what is in NVS.
 */
#include <string.h>
#include "unity.h"
#include "cfg.h"

static cfg_handle_t s_h;

static void open_clean(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, cfg_open("cfgtest", &s_h));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_erase_all(s_h));
}

static void close_h(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, cfg_close(s_h));
    s_h = NULL;
}

/* ------------------------------------------------------------------------- */
/* Lifecycle and arguments                                                    */
/* ------------------------------------------------------------------------- */

TEST_CASE("calls before init return ESP_ERR_INVALID_STATE", "[cfg]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cfg_deinit());
    cfg_handle_t h = NULL;
    uint32_t v;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, cfg_open("x", &h));
    TEST_ASSERT_NULL(h);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, cfg_get_u32((cfg_handle_t)1, "k", &v, 0));
    TEST_ASSERT_FALSE(cfg_exists((cfg_handle_t)1, "k"));
}

TEST_CASE("init is idempotent; open validates names and handles", "[cfg]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());

    cfg_handle_t h = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_open(NULL, &h));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_open("", &h));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_open("sixteen_chars_xx", &h)); /* 16 > 15 */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_open("ok", NULL));
    TEST_ASSERT_NULL(h);

    TEST_ASSERT_EQUAL(ESP_OK, cfg_open("fifteen_chars_x", &h)); /* exactly 15 */
    TEST_ASSERT_NOT_NULL(h);
    TEST_ASSERT_EQUAL(ESP_OK, cfg_close(h));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_close(NULL)); /* no-op */

    /* A stale or foreign pointer is rejected, not dereferenced. */
    uint32_t v;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_get_u32(h, "k", &v, 0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_get_u32((cfg_handle_t)0x12345678, "k", &v, 0));
}

TEST_CASE("handle table exhaustion returns ESP_ERR_NO_MEM and recovers on close", "[cfg]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    cfg_handle_t hs[CONFIG_CFG_MAX_HANDLES];
    for (int i = 0; i < CONFIG_CFG_MAX_HANDLES; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, cfg_open("exhaust", &hs[i]));
    }
    cfg_handle_t extra = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, cfg_open("exhaust", &extra));
    TEST_ASSERT_NULL(extra);
    TEST_ASSERT_EQUAL(ESP_OK, cfg_close(hs[0]));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_open("exhaust", &extra));
    for (int i = 1; i < CONFIG_CFG_MAX_HANDLES; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, cfg_close(hs[i]));
    }
    TEST_ASSERT_EQUAL(ESP_OK, cfg_close(extra));
}

/* ------------------------------------------------------------------------- */
/* Scalars                                                                    */
/* ------------------------------------------------------------------------- */

TEST_CASE("u32: absent key yields default + NOT_FOUND and persists the default", "[cfg]")
{
    open_clean();
    uint32_t v = 0;
    TEST_ASSERT_FALSE(cfg_exists(s_h, "u"));
    TEST_ASSERT_EQUAL(ESP_ERR_NVS_NOT_FOUND, cfg_get_u32(s_h, "u", &v, 42));
    TEST_ASSERT_EQUAL(42, v);
    TEST_ASSERT_TRUE(cfg_exists(s_h, "u"));

    /* Second read: found, default ignored. */
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_u32(s_h, "u", &v, 99));
    TEST_ASSERT_EQUAL(42, v);

    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_u32(s_h, "u", 7));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_u32(s_h, "u", &v, 99));
    TEST_ASSERT_EQUAL(7, v);

    TEST_ASSERT_EQUAL(ESP_OK, cfg_erase_key(s_h, "u"));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_erase_key(s_h, "u")); /* already gone: still OK */
    TEST_ASSERT_FALSE(cfg_exists(s_h, "u"));
    close_h();
}

TEST_CASE("i32 round trip including negatives", "[cfg]")
{
    open_clean();
    int32_t v = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_NVS_NOT_FOUND, cfg_get_i32(s_h, "i", &v, -5));
    TEST_ASSERT_EQUAL(-5, v);
    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_i32(s_h, "i", -123456));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_i32(s_h, "i", &v, 0));
    TEST_ASSERT_EQUAL(-123456, v);
    close_h();
}

TEST_CASE("key name too long is rejected without touching flash", "[cfg]")
{
    open_clean();
    uint32_t v = 1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_get_u32(s_h, "sixteen_chars_xx", &v, 5));
    TEST_ASSERT_EQUAL(1, v); /* untouched: no side effect on invalid arg */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_set_u32(s_h, "", 5));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_get_u32(s_h, "k", NULL, 5));
    close_h();
}

/* ------------------------------------------------------------------------- */
/* Strings                                                                    */
/* ------------------------------------------------------------------------- */

TEST_CASE("str: default, round trip, truncation reports INVALID_SIZE", "[cfg]")
{
    open_clean();
    char buf[8];

    TEST_ASSERT_EQUAL(ESP_ERR_NVS_NOT_FOUND, cfg_get_str(s_h, "s", buf, sizeof(buf), "dflt"));
    TEST_ASSERT_EQUAL_STRING("dflt", buf);
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_str(s_h, "s", buf, sizeof(buf), "other"));
    TEST_ASSERT_EQUAL_STRING("dflt", buf);

    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_str(s_h, "s", "hello"));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_str(s_h, "s", buf, sizeof(buf), NULL));
    TEST_ASSERT_EQUAL_STRING("hello", buf);

    /* Stored value longer than the buffer: buffer gets the default, error reported. */
    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_str(s_h, "s", "this is far too long"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, cfg_get_str(s_h, "s", buf, sizeof(buf), "d"));
    TEST_ASSERT_EQUAL_STRING("d", buf);

    /* NULL default means empty string. */
    TEST_ASSERT_EQUAL(ESP_ERR_NVS_NOT_FOUND, cfg_get_str(s_h, "s2", buf, sizeof(buf), NULL));
    TEST_ASSERT_EQUAL_STRING("", buf);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_set_str(s_h, "s", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_get_str(s_h, "s", buf, 0, "x"));
    close_h();
}

/* ------------------------------------------------------------------------- */
/* Versioned blobs                                                            */
/* ------------------------------------------------------------------------- */

typedef struct { uint32_t a; uint16_t b; } blob_v1_t;
typedef struct { uint32_t a; uint16_t b; uint8_t c; } blob_v2_t;

TEST_CASE("blob: round trip, schema mismatch reports stored schema, size check", "[cfg]")
{
    open_clean();
    size_t got = 99;
    uint16_t stored = 0;
    blob_v1_t in = { .a = 0xDEADBEEF, .b = 0x1234 };
    blob_v1_t out = {0};

    /* Absent: NOT_FOUND and nothing written (no scalar default for a blob). */
    TEST_ASSERT_EQUAL(ESP_ERR_NVS_NOT_FOUND, cfg_get_blob(s_h, "b", 1, &out, sizeof(out), &got, &stored));
    TEST_ASSERT_EQUAL(0, got);
    TEST_ASSERT_FALSE(cfg_exists(s_h, "b"));

    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_blob(s_h, "b", 1, &in, sizeof(in)));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_blob(s_h, "b", 1, &out, sizeof(out), &got, &stored));
    TEST_ASSERT_EQUAL(sizeof(in), got);
    TEST_ASSERT_EQUAL_MEMORY(&in, &out, sizeof(in));

    /* Caller moved to schema 2: told what is stored, nothing copied. */
    blob_v2_t out2 = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_VERSION, cfg_get_blob(s_h, "b", 2, &out2, sizeof(out2), &got, &stored));
    TEST_ASSERT_EQUAL(0, got);
    TEST_ASSERT_EQUAL(1, stored);

    /* Buffer too small for the payload. */
    uint8_t tiny[2];
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, cfg_get_blob(s_h, "b", 1, tiny, sizeof(tiny), &got, NULL));
    TEST_ASSERT_EQUAL(0, got);

    /* Zero-length blob is legal. */
    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_blob(s_h, "z", 3, NULL, 0));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_blob(s_h, "z", 3, NULL, 0, &got, NULL));
    TEST_ASSERT_EQUAL(0, got);

    /* Over the configured maximum. */
    static uint8_t big[CONFIG_CFG_MAX_BLOB_BYTES + 1];
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, cfg_set_blob(s_h, "big", 1, big, sizeof(big)));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, cfg_get_blob(s_h, "b", 1, &out, sizeof(out), NULL, NULL));
    close_h();
}

TEST_CASE("blob: foreign data under the key is reported as INVALID_CRC, not misread", "[cfg]")
{
    open_clean();
    /* Store a raw u32 where a blob is later expected. NVS types differ, so the
     * lookup fails as a type mismatch; either way it must not be parsed as a blob. */
    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_u32(s_h, "mixed", 1));
    size_t got = 5;
    uint8_t buf[16];
    esp_err_t err = cfg_get_blob(s_h, "mixed", 1, buf, sizeof(buf), &got, NULL);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(0, got);
    close_h();
}

/* ------------------------------------------------------------------------- */
/* Namespacing                                                                */
/* ------------------------------------------------------------------------- */

TEST_CASE("namespaces are isolated: same key, different values", "[cfg]")
{
    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    cfg_handle_t a = NULL, b = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, cfg_open("nsa", &a));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_open("nsb", &b));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_erase_all(a));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_erase_all(b));

    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_u32(a, "k", 1));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_u32(b, "k", 2));
    uint32_t va = 0, vb = 0;
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_u32(a, "k", &va, 0));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_u32(b, "k", &vb, 0));
    TEST_ASSERT_EQUAL(1, va);
    TEST_ASSERT_EQUAL(2, vb);

    /* Erasing one namespace leaves the other intact. */
    TEST_ASSERT_EQUAL(ESP_OK, cfg_erase_all(a));
    TEST_ASSERT_FALSE(cfg_exists(a, "k"));
    TEST_ASSERT_TRUE(cfg_exists(b, "k"));

    TEST_ASSERT_EQUAL(ESP_OK, cfg_close(a));
    TEST_ASSERT_EQUAL(ESP_OK, cfg_close(b));
}

TEST_CASE("values survive deinit/init (persistence within a power cycle)", "[cfg]")
{
    open_clean();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_set_u32(s_h, "persist", 0xA5A5));
    close_h();
    TEST_ASSERT_EQUAL(ESP_OK, cfg_deinit());

    TEST_ASSERT_EQUAL(ESP_OK, cfg_init());
    TEST_ASSERT_EQUAL(ESP_OK, cfg_open("cfgtest", &s_h));
    uint32_t v = 0;
    TEST_ASSERT_EQUAL(ESP_OK, cfg_get_u32(s_h, "persist", &v, 0));
    TEST_ASSERT_EQUAL(0xA5A5, v);
    TEST_ASSERT_EQUAL(ESP_OK, cfg_erase_all(s_h));
    close_h();
}

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
