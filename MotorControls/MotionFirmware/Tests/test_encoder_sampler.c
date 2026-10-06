#include "sim_diagnostics.h"
int main(void)
{
    encoder_sample_t s={0};
    encoder_sampler_health(&s,true,32U,64U,0U);
    assert(!encoder_sampler_accept(&s,0U,1U,2000U));
    encoder_sampler_config(&s,true,0x0300U);
    encoder_sampler_health(&s,true,32U,64U,0U);
    assert(encoder_sampler_accept(&s,4090U,1U,2000U));
    assert(encoder_sampler_accept(&s,5U,2001U,2000U) && s.unwrap.total_counts==11);
    assert(encoder_sampler_accept(&s,4090U,4001U,2000U) && s.unwrap.total_counts==0);
    assert(!encoder_sampler_accept(&s,10U,14002U,2000U) && (s.faults&PH_OVERRUN)!=0U);
    assert(!encoder_sampler_accept(&s,11U,14003U,2000U));
    memset(&s,0,sizeof(s));encoder_sampler_health(&s,true,32U,64U,0U);
    encoder_sampler_config(&s,true,0x0300U);
    assert(encoder_sampler_accept(&s,0U,1U,2000U));
    assert(!encoder_sampler_accept(&s,2048U,2001U,2000U) && s.unwrap.total_counts==0);
    memset(&s,0,sizeof(s));encoder_sampler_health(&s,true,32U,64U,0U);
    encoder_sampler_config(&s,true,0x0300U);
    assert(encoder_sampler_accept(&s,0U,1U,2000U));
    assert(!encoder_sampler_accept(&s,0U,2001U,400000U));
    memset(&s,0,sizeof(s));encoder_sampler_health(&s,true,0x30U,64U,0U);
    encoder_sampler_config(&s,true,0x0300U);
    assert(!encoder_sampler_accept(&s,4U,1U,2000U));
    sim_diagnostics_t sim;sim_init(&sim);sim.i2c_ok=false;
    sim.now+=2000U;encoder_sampler_poll(&sim.encoders,false);
    assert(!sim.encoders.axis[0].valid && (sim.encoders.axis[0].faults&PH_I2C)!=0U);
    sim.i2c_ok=true;sim.now+=2000U;encoder_sampler_poll(&sim.encoders,true);
    assert(!sim.encoders.axis[0].valid);
    assert(encoder_sampler_reseed(&sim.encoders,0U));
    assert(sim.encoders.axis[0].valid && !sim.encoders.axis[0].unwrap.suspect_alias);
    sim.encoders.axis[0].unwrap.total_counts=INT32_MAX;sim.raw[0]=1U;
    sim.now+=2000U;encoder_sampler_poll(&sim.encoders,false);
    assert(!sim.encoders.axis[0].valid && sim.encoders.axis[0].unwrap.total_counts==INT32_MAX);
    return 0;
}
