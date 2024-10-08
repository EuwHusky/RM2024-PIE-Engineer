/*
 * Copyright (c) 2022 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <math.h>
#include "hpm_math.h"

/**
 * @brief Basic arithmetic
 *
 */
#define hpm_mah_software_sinf(a)  sinf(a)

#define hpm_mah_software_cosf(a)  cosf(a)

/**
 * @brief Complex Structures
 *
 */
typedef struct
{
    float real;
    float imag;
} complex;

static complex hpm_math_sw_comp_add(complex x, complex y)
{
    complex c;
    c.real = x.real + y.real;
    c.imag = x.imag + y.imag;
    return c;
}

static complex hpm_math_sw_comp_sub(complex x, complex y)
{
    complex c;
    c.real = x.real - y.real;
    c.imag = x.imag - y.imag;
    return c;
}

static complex hpm_math_sw_comp_mul(complex x, complex y)
{
    complex c;
    c.real = x.real * y.real - x.imag * y.imag;
    c.imag = x.real * y.imag + x.imag * y.real;
    return c;
}

/**
 * @brief fft
 *
 */
#ifdef HPM_MATH_SW_FFT_CHECKLIST
/**
 * @brief Use the following code to generate arrays to support more sampling points
 *
 */
/*
#include <stdio.h>
#define FFT_MAX_POINT 10
#define FFT_MIN_POINT 2
#define BUFF_FORMAT "uint16_t"
int main()
{
	unsigned int memory[1<<FFT_MAX_POINT]={0};
	unsigned int len;
	unsigned int m;
	for(unsigned int j = FFT_MAX_POINT; j >= FFT_MIN_POINT; j--) {
		m = j;
		len = 1 << j;
		printf("%s hpm_math_sw_fft_tbl%d[%d] = {",BUFF_FORMAT,m,len);
		for (int i = 0; i < len; i++) {
			memory[i] = (memory[i >> 1] >> 1) | ((i & 1) << (m - 1));
			if(i%15 == 0){
				printf("\r\n");
			}
			printf("%d ",memory[i]);
			if(i != len-1){
				printf(",");
			}
		}
		printf("};\r\n");
	}
	printf("%s *hpm_math_sw_fft_str[%d] = {\r\n",BUFF_FORMAT,FFT_MAX_POINT);
	for(int j = 1; j <= FFT_MAX_POINT; j++) {
		len = 1 << j;
		if (j < FFT_MIN_POINT) {
			printf("NULL");
		} else {
			printf("(%s *)hpm_math_sw_fft_tbl%d",BUFF_FORMAT, j);
		}
		if(j != FFT_MAX_POINT){
			printf(",\r\n");
		}
	}
	printf("\r\n};\r\n");

   return 0;
}
*/

/**
 * @brief Defining it as a const type runs slower than defining it as a global variable.
 * You can define it as a const type and increase the speed by locating the data in ramfunc.
 *
 */
uint16_t hpm_math_sw_fft_tbl10[1024] = {
0 ,512 ,256 ,768 ,128 ,640 ,384 ,896 ,64 ,576 ,320 ,832 ,192 ,704 ,448 ,
960 ,32 ,544 ,288 ,800 ,160 ,672 ,416 ,928 ,96 ,608 ,352 ,864 ,224 ,736 ,
480 ,992 ,16 ,528 ,272 ,784 ,144 ,656 ,400 ,912 ,80 ,592 ,336 ,848 ,208 ,
720 ,464 ,976 ,48 ,560 ,304 ,816 ,176 ,688 ,432 ,944 ,112 ,624 ,368 ,880 ,
240 ,752 ,496 ,1008 ,8 ,520 ,264 ,776 ,136 ,648 ,392 ,904 ,72 ,584 ,328 ,
840 ,200 ,712 ,456 ,968 ,40 ,552 ,296 ,808 ,168 ,680 ,424 ,936 ,104 ,616 ,
360 ,872 ,232 ,744 ,488 ,1000 ,24 ,536 ,280 ,792 ,152 ,664 ,408 ,920 ,88 ,
600 ,344 ,856 ,216 ,728 ,472 ,984 ,56 ,568 ,312 ,824 ,184 ,696 ,440 ,952 ,
120 ,632 ,376 ,888 ,248 ,760 ,504 ,1016 ,4 ,516 ,260 ,772 ,132 ,644 ,388 ,
900 ,68 ,580 ,324 ,836 ,196 ,708 ,452 ,964 ,36 ,548 ,292 ,804 ,164 ,676 ,
420 ,932 ,100 ,612 ,356 ,868 ,228 ,740 ,484 ,996 ,20 ,532 ,276 ,788 ,148 ,
660 ,404 ,916 ,84 ,596 ,340 ,852 ,212 ,724 ,468 ,980 ,52 ,564 ,308 ,820 ,
180 ,692 ,436 ,948 ,116 ,628 ,372 ,884 ,244 ,756 ,500 ,1012 ,12 ,524 ,268 ,
780 ,140 ,652 ,396 ,908 ,76 ,588 ,332 ,844 ,204 ,716 ,460 ,972 ,44 ,556 ,
300 ,812 ,172 ,684 ,428 ,940 ,108 ,620 ,364 ,876 ,236 ,748 ,492 ,1004 ,28 ,
540 ,284 ,796 ,156 ,668 ,412 ,924 ,92 ,604 ,348 ,860 ,220 ,732 ,476 ,988 ,
60 ,572 ,316 ,828 ,188 ,700 ,444 ,956 ,124 ,636 ,380 ,892 ,252 ,764 ,508 ,
1020 ,2 ,514 ,258 ,770 ,130 ,642 ,386 ,898 ,66 ,578 ,322 ,834 ,194 ,706 ,
450 ,962 ,34 ,546 ,290 ,802 ,162 ,674 ,418 ,930 ,98 ,610 ,354 ,866 ,226 ,
738 ,482 ,994 ,18 ,530 ,274 ,786 ,146 ,658 ,402 ,914 ,82 ,594 ,338 ,850 ,
210 ,722 ,466 ,978 ,50 ,562 ,306 ,818 ,178 ,690 ,434 ,946 ,114 ,626 ,370 ,
882 ,242 ,754 ,498 ,1010 ,10 ,522 ,266 ,778 ,138 ,650 ,394 ,906 ,74 ,586 ,
330 ,842 ,202 ,714 ,458 ,970 ,42 ,554 ,298 ,810 ,170 ,682 ,426 ,938 ,106 ,
618 ,362 ,874 ,234 ,746 ,490 ,1002 ,26 ,538 ,282 ,794 ,154 ,666 ,410 ,922 ,
90 ,602 ,346 ,858 ,218 ,730 ,474 ,986 ,58 ,570 ,314 ,826 ,186 ,698 ,442 ,
954 ,122 ,634 ,378 ,890 ,250 ,762 ,506 ,1018 ,6 ,518 ,262 ,774 ,134 ,646 ,
390 ,902 ,70 ,582 ,326 ,838 ,198 ,710 ,454 ,966 ,38 ,550 ,294 ,806 ,166 ,
678 ,422 ,934 ,102 ,614 ,358 ,870 ,230 ,742 ,486 ,998 ,22 ,534 ,278 ,790 ,
150 ,662 ,406 ,918 ,86 ,598 ,342 ,854 ,214 ,726 ,470 ,982 ,54 ,566 ,310 ,
822 ,182 ,694 ,438 ,950 ,118 ,630 ,374 ,886 ,246 ,758 ,502 ,1014 ,14 ,526 ,
270 ,782 ,142 ,654 ,398 ,910 ,78 ,590 ,334 ,846 ,206 ,718 ,462 ,974 ,46 ,
558 ,302 ,814 ,174 ,686 ,430 ,942 ,110 ,622 ,366 ,878 ,238 ,750 ,494 ,1006 ,
30 ,542 ,286 ,798 ,158 ,670 ,414 ,926 ,94 ,606 ,350 ,862 ,222 ,734 ,478 ,
990 ,62 ,574 ,318 ,830 ,190 ,702 ,446 ,958 ,126 ,638 ,382 ,894 ,254 ,766 ,
510 ,1022 ,1 ,513 ,257 ,769 ,129 ,641 ,385 ,897 ,65 ,577 ,321 ,833 ,193 ,
705 ,449 ,961 ,33 ,545 ,289 ,801 ,161 ,673 ,417 ,929 ,97 ,609 ,353 ,865 ,
225 ,737 ,481 ,993 ,17 ,529 ,273 ,785 ,145 ,657 ,401 ,913 ,81 ,593 ,337 ,
849 ,209 ,721 ,465 ,977 ,49 ,561 ,305 ,817 ,177 ,689 ,433 ,945 ,113 ,625 ,
369 ,881 ,241 ,753 ,497 ,1009 ,9 ,521 ,265 ,777 ,137 ,649 ,393 ,905 ,73 ,
585 ,329 ,841 ,201 ,713 ,457 ,969 ,41 ,553 ,297 ,809 ,169 ,681 ,425 ,937 ,
105 ,617 ,361 ,873 ,233 ,745 ,489 ,1001 ,25 ,537 ,281 ,793 ,153 ,665 ,409 ,
921 ,89 ,601 ,345 ,857 ,217 ,729 ,473 ,985 ,57 ,569 ,313 ,825 ,185 ,697 ,
441 ,953 ,121 ,633 ,377 ,889 ,249 ,761 ,505 ,1017 ,5 ,517 ,261 ,773 ,133 ,
645 ,389 ,901 ,69 ,581 ,325 ,837 ,197 ,709 ,453 ,965 ,37 ,549 ,293 ,805 ,
165 ,677 ,421 ,933 ,101 ,613 ,357 ,869 ,229 ,741 ,485 ,997 ,21 ,533 ,277 ,
789 ,149 ,661 ,405 ,917 ,85 ,597 ,341 ,853 ,213 ,725 ,469 ,981 ,53 ,565 ,
309 ,821 ,181 ,693 ,437 ,949 ,117 ,629 ,373 ,885 ,245 ,757 ,501 ,1013 ,13 ,
525 ,269 ,781 ,141 ,653 ,397 ,909 ,77 ,589 ,333 ,845 ,205 ,717 ,461 ,973 ,
45 ,557 ,301 ,813 ,173 ,685 ,429 ,941 ,109 ,621 ,365 ,877 ,237 ,749 ,493 ,
1005 ,29 ,541 ,285 ,797 ,157 ,669 ,413 ,925 ,93 ,605 ,349 ,861 ,221 ,733 ,
477 ,989 ,61 ,573 ,317 ,829 ,189 ,701 ,445 ,957 ,125 ,637 ,381 ,893 ,253 ,
765 ,509 ,1021 ,3 ,515 ,259 ,771 ,131 ,643 ,387 ,899 ,67 ,579 ,323 ,835 ,
195 ,707 ,451 ,963 ,35 ,547 ,291 ,803 ,163 ,675 ,419 ,931 ,99 ,611 ,355 ,
867 ,227 ,739 ,483 ,995 ,19 ,531 ,275 ,787 ,147 ,659 ,403 ,915 ,83 ,595 ,
339 ,851 ,211 ,723 ,467 ,979 ,51 ,563 ,307 ,819 ,179 ,691 ,435 ,947 ,115 ,
627 ,371 ,883 ,243 ,755 ,499 ,1011 ,11 ,523 ,267 ,779 ,139 ,651 ,395 ,907 ,
75 ,587 ,331 ,843 ,203 ,715 ,459 ,971 ,43 ,555 ,299 ,811 ,171 ,683 ,427 ,
939 ,107 ,619 ,363 ,875 ,235 ,747 ,491 ,1003 ,27 ,539 ,283 ,795 ,155 ,667 ,
411 ,923 ,91 ,603 ,347 ,859 ,219 ,731 ,475 ,987 ,59 ,571 ,315 ,827 ,187 ,
699 ,443 ,955 ,123 ,635 ,379 ,891 ,251 ,763 ,507 ,1019 ,7 ,519 ,263 ,775 ,
135 ,647 ,391 ,903 ,71 ,583 ,327 ,839 ,199 ,711 ,455 ,967 ,39 ,551 ,295 ,
807 ,167 ,679 ,423 ,935 ,103 ,615 ,359 ,871 ,231 ,743 ,487 ,999 ,23 ,535 ,
279 ,791 ,151 ,663 ,407 ,919 ,87 ,599 ,343 ,855 ,215 ,727 ,471 ,983 ,55 ,
567 ,311 ,823 ,183 ,695 ,439 ,951 ,119 ,631 ,375 ,887 ,247 ,759 ,503 ,1015 ,
15 ,527 ,271 ,783 ,143 ,655 ,399 ,911 ,79 ,591 ,335 ,847 ,207 ,719 ,463 ,
975 ,47 ,559 ,303 ,815 ,175 ,687 ,431 ,943 ,111 ,623 ,367 ,879 ,239 ,751 ,
495 ,1007 ,31 ,543 ,287 ,799 ,159 ,671 ,415 ,927 ,95 ,607 ,351 ,863 ,223 ,
735 ,479 ,991 ,63 ,575 ,319 ,831 ,191 ,703 ,447 ,959 ,127 ,639 ,383 ,895 ,
255 ,767 ,511 ,1023 };
uint16_t hpm_math_sw_fft_tbl9[512] = {
0 ,256 ,128 ,384 ,64 ,320 ,192 ,448 ,32 ,288 ,160 ,416 ,96 ,352 ,224 ,
480 ,16 ,272 ,144 ,400 ,80 ,336 ,208 ,464 ,48 ,304 ,176 ,432 ,112 ,368 ,
240 ,496 ,8 ,264 ,136 ,392 ,72 ,328 ,200 ,456 ,40 ,296 ,168 ,424 ,104 ,
360 ,232 ,488 ,24 ,280 ,152 ,408 ,88 ,344 ,216 ,472 ,56 ,312 ,184 ,440 ,
120 ,376 ,248 ,504 ,4 ,260 ,132 ,388 ,68 ,324 ,196 ,452 ,36 ,292 ,164 ,
420 ,100 ,356 ,228 ,484 ,20 ,276 ,148 ,404 ,84 ,340 ,212 ,468 ,52 ,308 ,
180 ,436 ,116 ,372 ,244 ,500 ,12 ,268 ,140 ,396 ,76 ,332 ,204 ,460 ,44 ,
300 ,172 ,428 ,108 ,364 ,236 ,492 ,28 ,284 ,156 ,412 ,92 ,348 ,220 ,476 ,
60 ,316 ,188 ,444 ,124 ,380 ,252 ,508 ,2 ,258 ,130 ,386 ,66 ,322 ,194 ,
450 ,34 ,290 ,162 ,418 ,98 ,354 ,226 ,482 ,18 ,274 ,146 ,402 ,82 ,338 ,
210 ,466 ,50 ,306 ,178 ,434 ,114 ,370 ,242 ,498 ,10 ,266 ,138 ,394 ,74 ,
330 ,202 ,458 ,42 ,298 ,170 ,426 ,106 ,362 ,234 ,490 ,26 ,282 ,154 ,410 ,
90 ,346 ,218 ,474 ,58 ,314 ,186 ,442 ,122 ,378 ,250 ,506 ,6 ,262 ,134 ,
390 ,70 ,326 ,198 ,454 ,38 ,294 ,166 ,422 ,102 ,358 ,230 ,486 ,22 ,278 ,
150 ,406 ,86 ,342 ,214 ,470 ,54 ,310 ,182 ,438 ,118 ,374 ,246 ,502 ,14 ,
270 ,142 ,398 ,78 ,334 ,206 ,462 ,46 ,302 ,174 ,430 ,110 ,366 ,238 ,494 ,
30 ,286 ,158 ,414 ,94 ,350 ,222 ,478 ,62 ,318 ,190 ,446 ,126 ,382 ,254 ,
510 ,1 ,257 ,129 ,385 ,65 ,321 ,193 ,449 ,33 ,289 ,161 ,417 ,97 ,353 ,
225 ,481 ,17 ,273 ,145 ,401 ,81 ,337 ,209 ,465 ,49 ,305 ,177 ,433 ,113 ,
369 ,241 ,497 ,9 ,265 ,137 ,393 ,73 ,329 ,201 ,457 ,41 ,297 ,169 ,425 ,
105 ,361 ,233 ,489 ,25 ,281 ,153 ,409 ,89 ,345 ,217 ,473 ,57 ,313 ,185 ,
441 ,121 ,377 ,249 ,505 ,5 ,261 ,133 ,389 ,69 ,325 ,197 ,453 ,37 ,293 ,
165 ,421 ,101 ,357 ,229 ,485 ,21 ,277 ,149 ,405 ,85 ,341 ,213 ,469 ,53 ,
309 ,181 ,437 ,117 ,373 ,245 ,501 ,13 ,269 ,141 ,397 ,77 ,333 ,205 ,461 ,
45 ,301 ,173 ,429 ,109 ,365 ,237 ,493 ,29 ,285 ,157 ,413 ,93 ,349 ,221 ,
477 ,61 ,317 ,189 ,445 ,125 ,381 ,253 ,509 ,3 ,259 ,131 ,387 ,67 ,323 ,
195 ,451 ,35 ,291 ,163 ,419 ,99 ,355 ,227 ,483 ,19 ,275 ,147 ,403 ,83 ,
339 ,211 ,467 ,51 ,307 ,179 ,435 ,115 ,371 ,243 ,499 ,11 ,267 ,139 ,395 ,
75 ,331 ,203 ,459 ,43 ,299 ,171 ,427 ,107 ,363 ,235 ,491 ,27 ,283 ,155 ,
411 ,91 ,347 ,219 ,475 ,59 ,315 ,187 ,443 ,123 ,379 ,251 ,507 ,7 ,263 ,
135 ,391 ,71 ,327 ,199 ,455 ,39 ,295 ,167 ,423 ,103 ,359 ,231 ,487 ,23 ,
279 ,151 ,407 ,87 ,343 ,215 ,471 ,55 ,311 ,183 ,439 ,119 ,375 ,247 ,503 ,
15 ,271 ,143 ,399 ,79 ,335 ,207 ,463 ,47 ,303 ,175 ,431 ,111 ,367 ,239 ,
495 ,31 ,287 ,159 ,415 ,95 ,351 ,223 ,479 ,63 ,319 ,191 ,447 ,127 ,383 ,
255 ,511 };
uint16_t hpm_math_sw_fft_tbl8[256] = {
0 ,128 ,64 ,192 ,32 ,160 ,96 ,224 ,16 ,144 ,80 ,208 ,48 ,176 ,112 ,
240 ,8 ,136 ,72 ,200 ,40 ,168 ,104 ,232 ,24 ,152 ,88 ,216 ,56 ,184 ,
120 ,248 ,4 ,132 ,68 ,196 ,36 ,164 ,100 ,228 ,20 ,148 ,84 ,212 ,52 ,
180 ,116 ,244 ,12 ,140 ,76 ,204 ,44 ,172 ,108 ,236 ,28 ,156 ,92 ,220 ,
60 ,188 ,124 ,252 ,2 ,130 ,66 ,194 ,34 ,162 ,98 ,226 ,18 ,146 ,82 ,
210 ,50 ,178 ,114 ,242 ,10 ,138 ,74 ,202 ,42 ,170 ,106 ,234 ,26 ,154 ,
90 ,218 ,58 ,186 ,122 ,250 ,6 ,134 ,70 ,198 ,38 ,166 ,102 ,230 ,22 ,
150 ,86 ,214 ,54 ,182 ,118 ,246 ,14 ,142 ,78 ,206 ,46 ,174 ,110 ,238 ,
30 ,158 ,94 ,222 ,62 ,190 ,126 ,254 ,1 ,129 ,65 ,193 ,33 ,161 ,97 ,
225 ,17 ,145 ,81 ,209 ,49 ,177 ,113 ,241 ,9 ,137 ,73 ,201 ,41 ,169 ,
105 ,233 ,25 ,153 ,89 ,217 ,57 ,185 ,121 ,249 ,5 ,133 ,69 ,197 ,37 ,
165 ,101 ,229 ,21 ,149 ,85 ,213 ,53 ,181 ,117 ,245 ,13 ,141 ,77 ,205 ,
45 ,173 ,109 ,237 ,29 ,157 ,93 ,221 ,61 ,189 ,125 ,253 ,3 ,131 ,67 ,
195 ,35 ,163 ,99 ,227 ,19 ,147 ,83 ,211 ,51 ,179 ,115 ,243 ,11 ,139 ,
75 ,203 ,43 ,171 ,107 ,235 ,27 ,155 ,91 ,219 ,59 ,187 ,123 ,251 ,7 ,
135 ,71 ,199 ,39 ,167 ,103 ,231 ,23 ,151 ,87 ,215 ,55 ,183 ,119 ,247 ,
15 ,143 ,79 ,207 ,47 ,175 ,111 ,239 ,31 ,159 ,95 ,223 ,63 ,191 ,127 ,
255 };
uint16_t hpm_math_sw_fft_tbl7[128] = {
0 ,64 ,32 ,96 ,16 ,80 ,48 ,112 ,8 ,72 ,40 ,104 ,24 ,88 ,56 ,
120 ,4 ,68 ,36 ,100 ,20 ,84 ,52 ,116 ,12 ,76 ,44 ,108 ,28 ,92 ,
60 ,124 ,2 ,66 ,34 ,98 ,18 ,82 ,50 ,114 ,10 ,74 ,42 ,106 ,26 ,
90 ,58 ,122 ,6 ,70 ,38 ,102 ,22 ,86 ,54 ,118 ,14 ,78 ,46 ,110 ,
30 ,94 ,62 ,126 ,1 ,65 ,33 ,97 ,17 ,81 ,49 ,113 ,9 ,73 ,41 ,
105 ,25 ,89 ,57 ,121 ,5 ,69 ,37 ,101 ,21 ,85 ,53 ,117 ,13 ,77 ,
45 ,109 ,29 ,93 ,61 ,125 ,3 ,67 ,35 ,99 ,19 ,83 ,51 ,115 ,11 ,
75 ,43 ,107 ,27 ,91 ,59 ,123 ,7 ,71 ,39 ,103 ,23 ,87 ,55 ,119 ,
15 ,79 ,47 ,111 ,31 ,95 ,63 ,127 };
uint16_t hpm_math_sw_fft_tbl6[64] = {
0 ,32 ,16 ,48 ,8 ,40 ,24 ,56 ,4 ,36 ,20 ,52 ,12 ,44 ,28 ,
60 ,2 ,34 ,18 ,50 ,10 ,42 ,26 ,58 ,6 ,38 ,22 ,54 ,14 ,46 ,
30 ,62 ,1 ,33 ,17 ,49 ,9 ,41 ,25 ,57 ,5 ,37 ,21 ,53 ,13 ,
45 ,29 ,61 ,3 ,35 ,19 ,51 ,11 ,43 ,27 ,59 ,7 ,39 ,23 ,55 ,
15 ,47 ,31 ,63 };
uint16_t hpm_math_sw_fft_tbl5[32] = {
0 ,16 ,8 ,24 ,4 ,20 ,12 ,28 ,2 ,18 ,10 ,26 ,6 ,22 ,14 ,
30 ,1 ,17 ,9 ,25 ,5 ,21 ,13 ,29 ,3 ,19 ,11 ,27 ,7 ,23 ,
15 ,31 };
uint16_t hpm_math_sw_fft_tbl4[16] = {
0 ,8 ,4 ,12 ,2 ,10 ,6 ,14 ,1 ,9 ,5 ,13 ,3 ,11 ,7 ,
15 };
uint16_t hpm_math_sw_fft_tbl3[8] = {
0 ,4 ,2 ,6 ,1 ,5 ,3 ,7 };
uint16_t hpm_math_sw_fft_tbl2[4] = {
0 ,2 ,1 ,3 };
uint16_t *hpm_math_sw_fft_str[10] = {
NULL,
(uint16_t *)hpm_math_sw_fft_tbl2,
(uint16_t *)hpm_math_sw_fft_tbl3,
(uint16_t *)hpm_math_sw_fft_tbl4,
(uint16_t *)hpm_math_sw_fft_tbl5,
(uint16_t *)hpm_math_sw_fft_tbl6,
(uint16_t *)hpm_math_sw_fft_tbl7,
(uint16_t *)hpm_math_sw_fft_tbl8,
(uint16_t *)hpm_math_sw_fft_tbl9,
(uint16_t *)hpm_math_sw_fft_tbl10
};
#endif

void hpm_software_cfft_float(float *src, uint32_t m)
{
    uint32_t len;
    complex t;
    complex w = {1, 0};
    complex *c = (complex *)src;
    len = 1 << m;
#ifdef HPM_MATH_SW_FFT_CHECKLIST
    uint16_t *memory;
    memory = hpm_math_sw_fft_str[m-1];
#else
    uint32_t *memory;
    memory = (uint32_t *)(src + 2 * len);
    for (int i = 0; i < len; i++) {
        memory[i] = (memory[i >> 1] >> 1) | ((i & 1) << (m - 1));
    }
#endif

    for (uint32_t i = 0; i < len; i++) {
        if (i < memory[i]) {
            t = c[i];
            c[i] = c[memory[i]];
            c[memory[i]] = t;
        }
    }
    for (uint32_t n = 1; n < len; n <<= 1) {
        complex cn;
        complex wn;
        cn.real = hpm_mah_software_cosf(HPM_MATH_PI / n);
        cn.imag = -hpm_mah_software_sinf(HPM_MATH_PI / n);
        for (uint32_t rl = n << 1, j = 0; j < len; j += rl) {
            wn = w;
            for (uint32_t k = 0; k < n; k++, wn = hpm_math_sw_comp_mul(wn, cn)) {
                complex a = c[j + k], b = hpm_math_sw_comp_mul(wn, c[j + n + k]);
                c[j + k] = hpm_math_sw_comp_add(a, b);
                c[j + n + k] = hpm_math_sw_comp_sub(a, b);
            }
        }
    }
}

/**
 * @brief Bit reversal
 *
 */

const unsigned char bit_reverse_tbl[] =
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

uint8_t hpm_math_sw_reverse_bit8_lsb_to_msb(uint8_t lsb)
{
    return bit_reverse_tbl[lsb];
}

uint8_t hpm_math_sw_reverse_bit8_msb_to_lsb(uint8_t msb)
{
    return bit_reverse_tbl[msb];
}

uint32_t hpm_math_sw_reverse_bit32_lsb_to_msb(uint32_t lsb)
{
    uint8_t *p0 = (uint8_t *)&lsb;
    uint32_t val;
    uint8_t *p1 = (uint8_t *)&val;

    p1[3] = bit_reverse_tbl[p0[0]];
    p1[2] = bit_reverse_tbl[p0[1]];
    p1[1] = bit_reverse_tbl[p0[2]];
    p1[0] = bit_reverse_tbl[p0[3]];

    return val;
}

uint32_t hpm_math_sw_reverse_bit32_msb_to_lsb(uint32_t msb)
{
    uint8_t *p0 = (uint8_t *)&msb;
    uint32_t val;
    uint8_t *p1 = (uint8_t *)&val;

    p1[3] = bit_reverse_tbl[p0[0]];
    p1[2] = bit_reverse_tbl[p0[1]];
    p1[1] = bit_reverse_tbl[p0[2]];
    p1[0] = bit_reverse_tbl[p0[3]];

    return val;
}
