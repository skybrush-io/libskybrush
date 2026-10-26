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

/**
 * @file replay_main.c
 * @brief Runs a fuzz target on the files given on the command line.
 *
 * Used instead of libFuzzer when the compiler does not support it, so the fuzz
 * targets can still be run on a corpus (e.g., as a regression test).
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static int replay_file(const char* path)
{
    FILE* fp;
    uint8_t* data = 0;
    size_t size = 0, capacity = 0, num_read;

    fp = fopen(path, "rb");
    if (fp == 0) {
        fprintf(stderr, "cannot open %s\n", path);
        return 1;
    }

    do {
        if (size == capacity) {
            uint8_t* new_data;
            capacity = capacity ? capacity * 2 : 4096;
            new_data = realloc(data, capacity);
            if (new_data == 0) {
                free(data);
                fclose(fp);
                return 1;
            }
            data = new_data;
        }
        num_read = fread(data + size, 1, capacity - size, fp);
        size += num_read;
    } while (num_read > 0);

    fclose(fp);

    LLVMFuzzerTestOneInput(data, size);
    free(data);

    return 0;
}

int main(int argc, char* argv[])
{
    int i, result = 0;

    for (i = 1; i < argc; i++) {
        result |= replay_file(argv[i]);
    }

    return result;
}
