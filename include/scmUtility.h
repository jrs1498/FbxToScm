#ifndef SCMUTILITY_H
#define SCMUTILITY_H

#include "scmPreReq.h"

namespace scm
{
	class Utility
	{
	public:
		static int16_t FloatToEightEight(float x)
		{
			int16_t ret = (int16_t)(x * 256.0);
			return ret;
		}

		static float_t DegreesToNormalizedRotation(float_t x)
		{
			x = fmod(x, 360.0);
			if (x < 0.0)
				x += 360.0;
			return x / 360.0;
		}

		static float_t RadiansToNormalizedRotation(float_t x)
		{
			float twopi = 3.14159 * 2.0;
			x = fmod(x, twopi);
			if (x < 0.0)
				x += twopi;
			return x / twopi;
		}

		static uint8_t NormalizedFloatToUnsignedByte(float x)
		{
			if (x >= 1.0)
				return 0xFF;
			if (x <= 0.0)
				return 0x00;

			return (uint8_t)(x * 255.0);
		}

		static int8_t NormalizedFloatToByte(float x)
		{
			if (x >= 1.0)
				return 0x7F;
			if (x <= -1.0)
				return 0x80;

			return(int8_t)(x * 127.0);
		}
	};
}

#endif