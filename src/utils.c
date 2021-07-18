#include <skybrush/utils.h>

void sb_bounding_box_expand(sb_bounding_box_t *box, float offset)
{
    sb_interval_expand(&box->x, offset);
    sb_interval_expand(&box->y, offset);
    sb_interval_expand(&box->z, offset);
}

void sb_interval_expand(sb_interval_t *interval, float offset)
{
    interval->min -= offset;
    interval->max += offset;

    if (interval->max < interval->min)
    {
        interval->min = interval->max = interval->min + (interval->max - interval->min) / 2.0f;
    }
}