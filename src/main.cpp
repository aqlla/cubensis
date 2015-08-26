
/* ************************************************************************** *
 * Cubensis Control File													  *
 * Aquilla Sherrock - 16 May 2015											  *
 * ************************************************************************** *
 */

#include "util.h"
#include "cubensis.h"

#define DYNAMIC_ALLOC
//#define STATIC_ALLOC


#if defined(STATIC_ALLOC)
Cubensis cube;
#elif defined(DYNAMIC_ALLOC)
Cubensis* cube;
#endif


void setup()
{
#if defined(STATIC_ALLOC)
    cube.prepare();
    cube.calibrateSensors(1000);
    cube.start();
#elif defined(DYNAMIC_ALLOC)
    cube = new Cubensis{};
    cube->start_motors();
    cube->calibrate_sensors(5000);
    cube->start();
#endif
}

void loop()
{
#if defined(STATIC_ALLOC)
    cube.update();
#elif defined(DYNAMIC_ALLOC)
    cube->update();
#endif
}



