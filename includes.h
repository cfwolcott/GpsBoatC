#ifndef INCLUDES_H
#define INCLUDES_H

#include <sys/time.h>

// Following are "ports" of various system functions from the Arduino ecosystem 
// so that they can be used in Linux. i.e. the Raspberry Pi

#define PI 			3.1415926535897932384626433832795
#define HALF_PI 	1.5707963267948966192313216916398
#define TWO_PI 		6.283185307179586476925286766559
#define DEG_TO_RAD 	0.017453292519943295769236907684886
#define RAD_TO_DEG 	57.295779513082320876798154814105

#define min(a,b) 		((a)<(b)?(a):(b))
#define max(a,b) 		((a)>(b)?(a):(b))
#define abs(x) 			((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     	((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) 	((deg)*DEG_TO_RAD)
#define degrees(rad) 	((rad)*RAD_TO_DEG)
#define sq(x) 			((x)*(x))

// Port for millis()
#define millis()		TOOLS_millis()

typedef char			S8;
typedef unsigned char	U8;
typedef int				S16;
typedef unsigned int	U16;
typedef long			S32;
typedef unsigned long	U32;
typedef unsigned char	byte;

#include "tools.h"

#ifndef __cplusplus
typedef unsigned char bool;
#endif

#endif

