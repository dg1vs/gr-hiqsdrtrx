/*********************************************************************************
Header file for fd_filter.c
Created with MatLAB by create_filter_fd.m
$LastChangedDate: 2009-08-18 16:02:10 +0200 (Di, 18. Aug 2009) $
$LastChangedBy: norbertl $
$HeadURL: svn://svn.radiodata.xx/apg/branches/dipragw/Matlab/create_fd_filter.m $
$Revision: 466 $
Filters have base delay of 15 samples (1875.000000 ???s).
Total delay for a given filter fd_filter[N] is 15+N/128 samples (1875.000???s+N*1.953125???s).
Maximum errors calculated between 0 Hz and 3375.000 Hz
Maximum amplitude error 0.008 dB at 1500.000 Hz in 52/128 samples (101.562500 ???s) FD filter
Maximum phase delay error -124.020963 ???s at 3375.000 Hz in 127/128 samples (248.046875 ???s) FD filter
*********************************************************************************/
#ifndef FD_FILTER_H_
#define FD_FILTER_H_
#define MAX_FD_FILTER 128
#define FD_FILTER_LENGTH 32
extern const double fd_filter[MAX_FD_FILTER][FD_FILTER_LENGTH];
#endif
