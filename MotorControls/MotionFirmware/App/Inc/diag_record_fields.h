#ifndef DIAG_RECORD_FIELDS_H
#define DIAG_RECORD_FIELDS_H
/* Each expansion packs one named field at its specified wire offset. */
#define DIAG_PHASE_FIELDS(F) \
 F(u8,axis,0) F(u8,phase,1) F(u16,axis_flags,2) \
 F(i32,requested_target_01mm,4) F(i32,applied_target_01mm,8) \
 F(i32,start_position_steps,12) F(i32,target_position_steps,16) \
 F(i32,commanded_delta_steps,20) F(i32,emitted_delta_steps,24) \
 F(u32,emitted_edge_count,28) F(u32,step_angle_udeg,32) F(i32,quant_residual_nm,36) \
 F(u16,mscnt_start,40) F(u16,mscnt_end,42) F(u16,mscnt_expected,44) F(u16,mscnt_observed,46) \
 F(u8,mscnt_check,48) F(u8,driver_mode,49) F(u16,raw_start,50) F(u16,raw_end,52) \
 F(u8,as_status_start,54) F(u8,as_status_end,55) F(u8,agc_start,56) F(u8,agc_end,57) \
 F(u16,as_conf,58) F(i32,unwrap_start,60) F(i32,unwrap_end,64) F(i32,unwrap_delta,68) \
 F(i32,shaft_delta_001deg,72) F(i32,displacement_um,76) F(i32,home_offset_counts,80) \
 F(u32,move_duration_us,84) F(u32,max_sample_gap_us,88) \
 F(u32,raw_start_timestamp_us,92) F(u32,raw_end_timestamp_us,96) \
 F(u16,health_start_age_ms,100) F(u16,health_end_age_ms,102)
#define DIAG_HOME_FIELDS(F) \
 F(u8,axis,0) F(u8,result,1) F(u8,sg_threshold,3) F(i32,seek_emitted_steps,4) \
 F(u32,backoff_steps,8) F(i32,latch_emitted_steps,12) F(u16,sg_baseline,16) \
 F(u16,sg_trigger,18) F(u16,mscnt_zero,20) F(u16,raw_zero,22) \
 F(i32,home_offset_counts,24) F(i32,home_emitted_origin_steps,28) \
 F(u32,home_duration_us,32) F(u16,axis_flags,36) F(u8,as_status,38) F(u8,agc,39)
#define DIAG_STATUS_FIELDS(F) \
 F(u8,state,0) F(u8,command_queue_depth,1) F(u8,tx_queue_depth,2) F(u8,homed_mask,3) \
 F(u32,rx_crc_errors,4) F(u32,rx_format_errors,8) F(u32,rx_overflows,12) \
 F(u32,rejected_commands,16) F(u32,tx_record_drops,20) \
 F(u32,x_drv_status,24) F(u32,z_drv_status,28) \
 F(i32,x_position_steps,32) F(i32,z_position_steps,36) \
 F(i32,x_unwrap_counts,40) F(i32,z_unwrap_counts,44) \
 F(u32,x_max_sample_gap_us,48) F(u32,z_max_sample_gap_us,52) \
 F(u8,x_as_status,56) F(u8,z_as_status,57) F(u8,x_agc,58) F(u8,z_agc,59) \
 F(i32,x_home_offset_counts,60) F(i32,z_home_offset_counts,64) \
 F(u16,x_health_age_ms,68) F(u16,z_health_age_ms,70)
#endif
