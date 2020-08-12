/*****************************************************************
	String Prediction
*****************************************************************/
1. To turn on & off String Prediction

  To turn on String Prediction, scc coding tool "ibc" and "sp" must be turned on in configuration files.

*****************************************************************
2. Added files

  String Prediction adds 3 extra files.

  src\enc_sm.cpp

  inc\com_usp.h
  inc\enc_sm.h

*****************************************************************
3. Files modified

  The following 23 files are modified:

  src\enc_pintra.c
  src\enc_pinter.c
  src\enc_mode.c
  src\enc_eco.c
  src\enc.c
  src\dec_util.c
  src\dec_eco.c
  src\dec.c
  src\com_util.c
  src\com_recon.c
  src\com_df.c
  
  inc\enc_pintra.h
  inc\enc_mode.h
  inc\enc_eco.h
  inc\enc_def.h
  inc\dec_def.h
  inc\com_util.h
  inc\com_typedef.h
  inc\com_recon.h
  inc\com_def.h
  
  app\app_encoder.c
  app\app_decoder.c
  app\app_bitstream_merge.c

*****************************************************************
4. Other modification
 
  When macro "USE_SP" is set to 1, two global variables "g_CountDOICyCleTime" and "g_DOIPrev" are moved from "com_typedef.h" to "app_decoder.c","app_encoder.c", "app_bitstream_merge.c" to avoid LNK2005 error.
 
  When macro "USE_SP" is set to 0, two global variables "g_CountDOICyCleTime" and "g_DOIPrev" are still in "com_typedef.h", thus Visual Studio generates two "LNK4006" warnings.
******************************************************************