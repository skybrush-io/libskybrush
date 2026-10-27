/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2026 CollMot Robotics Ltd.
 *
 * libskybrush is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * libskybrush is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <string.h>

#include "trace_output.h"

FILE* out;

void print_float(float value)
{
    if (isfinite(value)) {
        /* Nine significant digits are enough to represent any float exactly */
        fprintf(out, "%.9g", (double)value);
    } else {
        fprintf(out, "null");
    }
}

void print_vector3(sb_vector3_t vec)
{
    fprintf(out, "[");
    print_float(vec.x);
    fprintf(out, ", ");
    print_float(vec.y);
    fprintf(out, ", ");
    print_float(vec.z);
    fprintf(out, "]");
}

void print_vector3_with_yaw(sb_vector3_with_yaw_t vec)
{
    sb_vector3_t vec3 = { vec.x, vec.y, vec.z };
    print_vector3(vec3);
}

void print_color(sb_rgb_color_t color)
{
    fprintf(out, "[%d, %d, %d]", color.red, color.green, color.blue);
}

void print_hex(const uint8_t* bytes, size_t length)
{
    fprintf(out, "\"");
    for (size_t i = 0; i < length; i++) {
        fprintf(out, "%02x", bytes[i]);
    }
    fprintf(out, "\"");
}

void print_error(sb_error_t error)
{
    static const char* names[] = {
        "SUCCESS", "ENOMEM", "EINVAL", "EOPEN", "ECLOSE", "EREAD", "EWRITE",
        "EREADWRITE", "EPARSE", "TIMEOUT", "ELOCKED", "FAILURE", "EUNSUPPORTED",
        "EUNIMPLEMENTED", "EPERM", "EFULL", "EEMPTY", "EAGAIN", "ENOENT",
        "ECORRUPTED", "EOVERFLOW"
    };

    if ((size_t)error < sizeof(names) / sizeof(names[0])) {
        fprintf(out, "\"%s\"", names[error]);
    } else {
        fprintf(out, "\"UNKNOWN\"");
    }
}

void print_item_separator(uint32_t index, uint32_t length)
{
    fprintf(out, index + 1 < length ? ",\n      " : "\n    ");
}

void read_show_file(const char* path, uint8_t** data, size_t* length)
{
    FILE* fp = fopen(path, "rb");
    long file_length;

    if (fp == 0) {
        perror(path);
        exit(1);
    }

    if (fseek(fp, 0, SEEK_END) != 0 || (file_length = ftell(fp)) < 0 || fseek(fp, 0, SEEK_SET) != 0) {
        perror(path);
        exit(1);
    }

    *length = (size_t)file_length;
    *data = malloc(*length > 0 ? *length : 1);
    if (*data == 0 || fread(*data, 1, *length, fp) != *length) {
        perror(path);
        exit(1);
    }

    fclose(fp);
}

const char* base_name(const char* path)
{
    const char* slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

int is_present(sb_error_t retval, const char* component)
{
    if (retval == SB_ENOENT) {
        return 0;
    } else if (retval != SB_SUCCESS) {
        fprintf(stderr, "failed to load %s: %s\n", component, sb_error_to_string(retval));
        exit(1);
    }
    return 1;
}
