/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 by libskybrush authors. See AUTHORS.
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

#include <skybrush/utils.h>

void sb_bounding_box_expand(sb_bounding_box_t* box, float offset)
{
    sb_interval_expand(&box->x, offset);
    sb_interval_expand(&box->y, offset);
    sb_interval_expand(&box->z, offset);
}

void sb_interval_expand(sb_interval_t* interval, float offset)
{
    interval->min -= offset;
    interval->max += offset;

    if (interval->max < interval->min) {
        interval->min = interval->max = interval->min + (interval->max - interval->min) / 2.0f;
    }
}
