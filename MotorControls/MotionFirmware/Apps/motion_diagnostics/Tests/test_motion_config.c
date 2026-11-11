#include "motion_config.h"
#include <assert.h>
#include <limits.h>
int main(void)
{
    assert(motion_config_valid(&motion_default_config));
    int32_t steps,residual;
    for (int32_t t=-32768;t<=32767;++t) {
        assert(motion_target_steps(&motion_default_config.axis[AXIS_X],t,&steps,&residual));
        assert(steps==8*t && residual==0);
        assert(motion_target_steps(&motion_default_config.axis[AXIS_Z],t,&steps,&residual));
        assert(steps==40*t && residual==0);
    }
    motion_axis_config_t a=motion_default_config.axis[AXIS_X];a.lead_um=640000U;
    assert(motion_target_steps(&a,1,&steps,&residual) && steps==1 && residual==100000);
    assert(motion_target_steps(&a,-1,&steps,&residual) && steps==-1 && residual==-100000);
    assert(!motion_target_steps(&motion_default_config.axis[AXIS_Z],INT32_MAX,&steps,&residual));
    motion_config_t c=motion_default_config;c.pick_depth_01mm=-1;assert(!motion_config_valid(&c));
    c=motion_default_config;c.pick_depth_01mm=1401;assert(!motion_config_valid(&c));
    c=motion_default_config;c.axis[0].max_rate=2001U;assert(!motion_config_valid(&c));
    assert(!motion_default_config.bounds_confirmed && motion_default_config.pick_depth_01mm==0);
    return 0;
}
