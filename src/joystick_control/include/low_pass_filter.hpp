#include <iostream>
#include <vector>

class LowPassFilter
{
private:
    std::vector<double> buffer;
    double alpha;

public:
    LowPassFilter(double alpha) : alpha(alpha) {}

    double filter(double value)
    {
        if (buffer.empty())
        {
            buffer.push_back(value);
            return value;
        }

        double filteredValue = alpha * value + (1 - alpha) * buffer.back();
        buffer.push_back(filteredValue);
        return filteredValue;
    }

    void clear()
    {
        buffer.clear();
    }
};