#include "utils.h"

float eucl2(float x1, float y1, float x2, float y2)
{
	return sqrtf(powf(x2 - x1, 2.0f) + powf(y2 - y1, 2.0f));
}
