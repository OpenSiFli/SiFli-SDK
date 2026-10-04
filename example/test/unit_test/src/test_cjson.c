/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>

#include "unity.h"
#include "cJSON.h"

static const char *const document =
    "{"
    "\"name\":\"sifli\","
    "\"version\":2,"
    "\"enabled\":true,"
    "\"tags\":[\"a\",\"b\"],"
    "\"nested\":{\"ok\":true}"
    "}";

/* A well-formed document: every value must be readable back with the type it
 * was written as. */
void test_cjson_parse_and_read(void)
{
    cJSON *root;
    cJSON *item;

    root = cJSON_Parse(document);
    TEST_ASSERT_NOT_NULL(root);
    TEST_ASSERT_TRUE(cJSON_IsObject(root));

    item = cJSON_GetObjectItemCaseSensitive(root, "name");
    TEST_ASSERT_TRUE(cJSON_IsString(item));
    TEST_ASSERT_EQUAL_STRING("sifli", cJSON_GetStringValue(item));

    item = cJSON_GetObjectItemCaseSensitive(root, "version");
    TEST_ASSERT_TRUE(cJSON_IsNumber(item));
    /* Compared as an integer: this Unity build has no double support. */
    TEST_ASSERT_EQUAL_INT(2, (int)cJSON_GetNumberValue(item));

    item = cJSON_GetObjectItemCaseSensitive(root, "enabled");
    TEST_ASSERT_TRUE(cJSON_IsTrue(item));

    item = cJSON_GetObjectItemCaseSensitive(root, "tags");
    TEST_ASSERT_TRUE(cJSON_IsArray(item));
    TEST_ASSERT_EQUAL_INT(2, cJSON_GetArraySize(item));
    TEST_ASSERT_EQUAL_STRING("a", cJSON_GetStringValue(cJSON_GetArrayItem(item, 0)));
    TEST_ASSERT_EQUAL_STRING("b", cJSON_GetStringValue(cJSON_GetArrayItem(item, 1)));

    item = cJSON_GetObjectItemCaseSensitive(root, "nested");
    TEST_ASSERT_TRUE(cJSON_IsObject(item));
    TEST_ASSERT_TRUE(cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(item, "ok")));

    cJSON_Delete(root);
}

/* Malformed input has no partial success: the parser must report NULL. */
/* case: malformed */
void test_cjson_malformed_returns_null(void)
{
    TEST_ASSERT_NULL(cJSON_Parse(""));
    TEST_ASSERT_NULL(cJSON_Parse("not json at all"));
    TEST_ASSERT_NULL(cJSON_Parse("{"));
    TEST_ASSERT_NULL(cJSON_Parse("{\"name\":}"));
    TEST_ASSERT_NULL(cJSON_Parse("{\"name\" \"sifli\"}"));
    TEST_ASSERT_NULL(cJSON_Parse("[1,2"));
}

/* Lookups at and past the edges: an absent key, an out-of-range index, and a
 * value asked for as the wrong type. */
/* case: boundaries */
void test_cjson_boundary_lookups(void)
{
    cJSON *root;
    cJSON *tags;
    cJSON *item;

    root = cJSON_Parse("{\"tags\":[\"a\",\"b\"],\"version\":2}");
    TEST_ASSERT_NOT_NULL(root);

    tags = cJSON_GetObjectItemCaseSensitive(root, "tags");
    TEST_ASSERT_EQUAL_INT(2, cJSON_GetArraySize(tags));

    /* One past the last element, and one before the first. */
    TEST_ASSERT_NULL(cJSON_GetArrayItem(tags, 2));
    TEST_ASSERT_NULL(cJSON_GetArrayItem(tags, -1));

    /* A key that is not there. */
    TEST_ASSERT_NULL(cJSON_GetObjectItemCaseSensitive(root, "absent"));

    /* A string accessor on a number, and the array accessors on a scalar:
     * the type checks must refuse rather than reinterpret. */
    item = cJSON_GetObjectItemCaseSensitive(root, "version");
    TEST_ASSERT_TRUE(cJSON_IsNumber(item));
    TEST_ASSERT_FALSE(cJSON_IsString(item));
    TEST_ASSERT_NULL(cJSON_GetStringValue(item));
    TEST_ASSERT_EQUAL_INT(0, cJSON_GetArraySize(item));
    TEST_ASSERT_NULL(cJSON_GetArrayItem(item, 0));

    cJSON_Delete(root);
}
