#include <follow_me/common/utils.h>

float findFwdDist(const std::vector<float>& ranges, const std::vector<float>& thetas)
{
    // Simple average filter over the front rays in case a few are invalid.
    int num_range = 5;
    int num_scans = 0;
    float sum = 0;

    for (int i = 0; i < num_range; ++i)
    {
        if (ranges[i] > 0)
        {
            sum += ranges[i] * cos(thetas[i]);
            num_scans++;
        }
    }
    for (int i = ranges.size() - num_range; i < ranges.size(); ++i)
    {
        if (ranges[i] > 0)
        {
            sum += ranges[i] * cos(thetas[i]);
            num_scans++;
        }
    }

    if (num_scans < 1) return -1;

    return sum / num_scans;
}
