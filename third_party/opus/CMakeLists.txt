# Vendored libopus 1.5.2, fixed-point, no float API, no DNN extensions (ADR-024).
# Upstream builds with autotools/CMake/meson and a generated config.h; this file
# replaces all three with the handful of definitions the sources actually test.
# The source lists mirror upstream's celt_sources.mk, silk_sources.mk (SILK_SOURCES +
# SILK_SOURCES_FIXED) and opus_sources.mk minus multistream/projection, with every
# architecture-specific (x86/ARM/MIPS) file left out: the LX7 has none.

set(celt_srcs
    "celt/bands.c"
    "celt/celt.c"
    "celt/celt_decoder.c"
    "celt/celt_encoder.c"
    "celt/celt_lpc.c"
    "celt/cwrs.c"
    "celt/entcode.c"
    "celt/entdec.c"
    "celt/entenc.c"
    "celt/kiss_fft.c"
    "celt/laplace.c"
    "celt/mathops.c"
    "celt/mdct.c"
    "celt/modes.c"
    "celt/pitch.c"
    "celt/quant_bands.c"
    "celt/rate.c"
    "celt/vq.c"
)
set(silk_srcs
    "silk/A2NLSF.c"
    "silk/CNG.c"
    "silk/HP_variable_cutoff.c"
    "silk/LPC_analysis_filter.c"
    "silk/LPC_fit.c"
    "silk/LPC_inv_pred_gain.c"
    "silk/LP_variable_cutoff.c"
    "silk/NLSF2A.c"
    "silk/NLSF_VQ.c"
    "silk/NLSF_VQ_weights_laroia.c"
    "silk/NLSF_decode.c"
    "silk/NLSF_del_dec_quant.c"
    "silk/NLSF_encode.c"
    "silk/NLSF_stabilize.c"
    "silk/NLSF_unpack.c"
    "silk/NSQ.c"
    "silk/NSQ_del_dec.c"
    "silk/PLC.c"
    "silk/VAD.c"
    "silk/VQ_WMat_EC.c"
    "silk/ana_filt_bank_1.c"
    "silk/biquad_alt.c"
    "silk/bwexpander.c"
    "silk/bwexpander_32.c"
    "silk/check_control_input.c"
    "silk/code_signs.c"
    "silk/control_SNR.c"
    "silk/control_audio_bandwidth.c"
    "silk/control_codec.c"
    "silk/debug.c"
    "silk/dec_API.c"
    "silk/decode_core.c"
    "silk/decode_frame.c"
    "silk/decode_indices.c"
    "silk/decode_parameters.c"
    "silk/decode_pitch.c"
    "silk/decode_pulses.c"
    "silk/decoder_set_fs.c"
    "silk/enc_API.c"
    "silk/encode_indices.c"
    "silk/encode_pulses.c"
    "silk/gain_quant.c"
    "silk/init_decoder.c"
    "silk/init_encoder.c"
    "silk/inner_prod_aligned.c"
    "silk/interpolate.c"
    "silk/lin2log.c"
    "silk/log2lin.c"
    "silk/pitch_est_tables.c"
    "silk/process_NLSFs.c"
    "silk/quant_LTP_gains.c"
    "silk/resampler.c"
    "silk/resampler_down2.c"
    "silk/resampler_down2_3.c"
    "silk/resampler_private_AR2.c"
    "silk/resampler_private_IIR_FIR.c"
    "silk/resampler_private_down_FIR.c"
    "silk/resampler_private_up2_HQ.c"
    "silk/resampler_rom.c"
    "silk/shell_coder.c"
    "silk/sigm_Q15.c"
    "silk/sort.c"
    "silk/stereo_LR_to_MS.c"
    "silk/stereo_MS_to_LR.c"
    "silk/stereo_decode_pred.c"
    "silk/stereo_encode_pred.c"
    "silk/stereo_find_predictor.c"
    "silk/stereo_quant_pred.c"
    "silk/sum_sqr_shift.c"
    "silk/table_LSF_cos.c"
    "silk/tables_LTP.c"
    "silk/tables_NLSF_CB_NB_MB.c"
    "silk/tables_NLSF_CB_WB.c"
    "silk/tables_gain.c"
    "silk/tables_other.c"
    "silk/tables_pitch_lag.c"
    "silk/tables_pulses_per_block.c"
)
set(silk_fixed_srcs
    "silk/fixed/LTP_analysis_filter_FIX.c"
    "silk/fixed/LTP_scale_ctrl_FIX.c"
    "silk/fixed/apply_sine_window_FIX.c"
    "silk/fixed/autocorr_FIX.c"
    "silk/fixed/burg_modified_FIX.c"
    "silk/fixed/corrMatrix_FIX.c"
    "silk/fixed/encode_frame_FIX.c"
    "silk/fixed/find_LPC_FIX.c"
    "silk/fixed/find_LTP_FIX.c"
    "silk/fixed/find_pitch_lags_FIX.c"
    "silk/fixed/find_pred_coefs_FIX.c"
    "silk/fixed/k2a_FIX.c"
    "silk/fixed/k2a_Q16_FIX.c"
    "silk/fixed/noise_shape_analysis_FIX.c"
    "silk/fixed/pitch_analysis_core_FIX.c"
    "silk/fixed/process_gains_FIX.c"
    "silk/fixed/regularize_correlations_FIX.c"
    "silk/fixed/residual_energy16_FIX.c"
    "silk/fixed/residual_energy_FIX.c"
    "silk/fixed/schur64_FIX.c"
    "silk/fixed/schur_FIX.c"
    "silk/fixed/vector_ops_FIX.c"
    "silk/fixed/warped_autocorrelation_FIX.c"
)
set(opus_srcs
    "src/opus.c"
    "src/opus_decoder.c"
    "src/opus_encoder.c"
    "src/extensions.c"
    "src/repacketizer.c"
)

idf_component_register(SRCS ${celt_srcs} ${silk_srcs} ${silk_fixed_srcs} ${opus_srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "." "celt" "silk" "silk/fixed" "src")

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    OPUS_BUILD
    FIXED_POINT=1          # the codec state and PCM are int16/int32 throughout
    DISABLE_FLOAT_API      # opus_encode()/opus_decode() only; no *_float entry points
    VAR_ARRAYS             # C99 VLAs for scratch: stack, not a global pseudo-stack
    HAVE_LRINT=1           # newlib has both; without them float_cast.h falls back
    HAVE_LRINTF=1          # to a plain cast and warns
    PACKAGE_VERSION="1.5.2"
)

# The codec is the hot path of every wireless frame (DES-LNK-004): build it for speed
# whatever the project's optimisation level. Upstream's own flags do the same.
# Third-party code: not held to sdk_component_strict() (SYS-BLD-005 is for SDK code).
# The suppressions are exactly the warnings upstream 1.5.2 emits under ESP-IDF's
# flags: src/repacketizer.c sets a variable used only under an assert, and GCC 15 at
# -O2 cannot see through celt_lpc.c's variable-length arrays (a false positive).
target_compile_options(${COMPONENT_LIB} PRIVATE
    -O2
    -Wno-unused-variable
    -Wno-maybe-uninitialized
)
