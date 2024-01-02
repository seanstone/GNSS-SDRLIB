/*------------------------------------------------------------------------------
* sdrmain.c : SDR main functions
*
* Copyright (C) 2014 Taro Suzuki <gnsssdrlib@gmail.com>
*-----------------------------------------------------------------------------*/
#include "sdr.h"
#include <chrono>
#include <time.h>
#include <locale.h>

/* global variables ----------------------------------------------------------*/
#ifdef GUI
GCHandle hform;
#endif

/* thread handle and mutex */
thread_t hmainthread;
thread_t hsyncthread;
thread_t hspecthread;
thread_t hkeythread;
mlock_t hbuffmtx;
mlock_t hreadmtx;
mlock_t hfftmtx;
mlock_t hpltmtx;
mlock_t hobsmtx;
mlock_t hlexmtx;
event_t hlexeve;

/* sdr structs */
sdrini_t sdrini={0};
sdrstat_t sdrstat={0};
sdrch_t sdrch[MAXSAT]={{0}};
sdrspec_t sdrspec={0};
sdrout_t sdrout={0};

/* initsdrgui ------------------------------------------------------------------
* initialize sdr gui application  
* args   : maindlg^ form       I   main dialog class
*          sdrini_t* sdrinigui I   sdr init struct
* return : none
* note : This function is only used in GUI application 
*-----------------------------------------------------------------------------*/
#ifdef GUI
extern void initsdrgui(maindlg^ form, sdrini_t* sdrinigui)
{
    /* initialization global structs */
    memset(&sdrini,0,sizeof(sdrini));
    memset(&sdrstat,0,sizeof(sdrstat));
    memset(&sdrch,0,sizeof(sdrch));

    hform=GCHandle::Alloc(form);
    memcpy(&sdrini,sdrinigui,sizeof(sdrini_t)); /* copy setting from GUI */
}
#else
/* keyboard thread -------------------------------------------------------------
* keyboard thread for program termination  
* args   : void   *arg      I   not used
* return : none
* note : this thread is only created in CLI application
*-----------------------------------------------------------------------------*/
#ifdef WIN32
extern void keythread(void * arg) 
#else
extern void *keythread(void * arg) 
#endif
{
    do {
        switch(getchar()) {
        case 'q':
        case 'Q':
            sdrstat.stopflag=1;
            break;
        default:
            SDRPRINTF("press 'q' to exit...\n");
            break;
        }
    } while (!sdrstat.stopflag);

    return THRETVAL;
}
/* main function ---------------------------------------------------------------
* main entry point in CLI application  
* args   : none
* return : none
* note : This function is only used in CLI application 
*-----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    /* read ini file */
    if (readinifile(&sdrini)<0) {
        return -1; 
    }
    cratethread(hkeythread,keythread,NULL);

    startsdr();

    return 0;
}
#endif
/* sdr start -------------------------------------------------------------------
* start sdr function  
* args   : void   *arg      I   not used
* return : none
* note : This function is called as thread in GUI application and is called as
*        function in CLI application
*-----------------------------------------------------------------------------*/
#ifdef GUI
extern void startsdr(void *arg) /* call as thread */
#else
extern void startsdr(void) /* call as function */
#endif
{
    int i;
    SDRPRINTF("GNSS-SDRLIB start!\n");

	setlocale(LC_ALL, "en-US");

    /* check initial value */
    if (chk_initvalue(&sdrini)<0) {
        SDRPRINTF("error: chk_initvalue\n");
        quitsdr(&sdrini,1);
        return;
    }

    /* receiver initialization */
    if (rcvinit(&sdrini)<0) {
        SDRPRINTF("error: rcvinit\n");
        quitsdr(&sdrini,1);
        return;
    }
    /* initialize sdr channel struct */
    for (i=0;i<sdrini.nch;i++) {
        if (initsdrch(i+1,sdrini.sys[i],sdrini.prn[i],sdrini.ctype[i],
            sdrini.dtype[sdrini.ftype[i]-1],sdrini.ftype[i],
            sdrini.f_cf[sdrini.ftype[i]-1],sdrini.f_sf[sdrini.ftype[i]-1],
            sdrini.f_if[sdrini.ftype[i]-1],&sdrch[i])<0) {
            
            SDRPRINTF("error: initsdrch\n");
            quitsdr(&sdrini,2);
            return;
        }
    }

    /* mutexes and events */
    openhandles();

    /* create threads */
    cratethread(hsyncthread,syncthread,NULL); /* synchronization thread */

    /* sdr channel thread */
    for (i=0;i<sdrini.nch;i++) {
        /* GPS/QZS/GLO/GAL/CMP L1 */
        if (sdrch[i].ctype==CTYPE_L1CA  || sdrch[i].ctype==CTYPE_G1  ||
            sdrch[i].ctype==CTYPE_E1B   || sdrch[i].ctype==CTYPE_B1I ||
            sdrch[i].ctype==CTYPE_L1SBAS|| sdrch[i].ctype==CTYPE_L1SAIF)
            cratethread(sdrch[i].hsdr,sdrthread,&sdrch[i]);
        /* QZSS LEX */
        if (sdrch[i].sys==SYS_QZS&&sdrch[i].ctype==CTYPE_LEXS) {
            sdrini.nchL6++;
            cratethread(sdrch[i].hsdr,lexthread,&sdrch[i]);

            /* create QZSS L1CA channel */
            initsdrch(sdrini.nch+1,SYS_QZS,193,CTYPE_L1CA,DTYPEI,FTYPE1,
               sdrini.f_cf[0],sdrini.f_sf[0],sdrini.f_if[0],&sdrch[sdrini.nch]);
            cratethread(sdrch[sdrini.nch].hsdr,sdrthread,&sdrch[sdrini.nch]);
        }
    }
#ifndef GUI
    /* sdr spectrum analyzer */
    if (sdrini.pltspec) {
        sdrspec.dtype=sdrini.dtype[sdrini.pltspec-1];
        sdrspec.ftype=sdrini.ftype[sdrini.pltspec-1];
        sdrspec.nsamp=(int)(sdrini.f_sf[sdrini.pltspec-1]/1000); /* 1ms */
        sdrspec.f_sf=sdrini.f_sf[sdrini.pltspec-1];
        cratethread(hspecthread,specthread,&sdrspec);
    }
#endif


    /* data grabber loop */
    while (!sdrstat.stopflag) 
	{
        if (rcvgrabdata(&sdrini) < 0) 
		{
            sdrstat.stopflag=ON;
            break;
        }
    }
    /* wait thereds */
    waitthread(hsyncthread);
    for (i=0;i<sdrini.nch;i++) 
        waitthread(sdrch[i].hsdr);

    /* sdr termination */
    quitsdr(&sdrini,0);

    SDRPRINTF("GNSS-SDRLIB is finished!\n");
}
/* sdr termination -------------------------------------------------------------
* sdr termination process  
* args   : sdrini_t *ini    I   sdr init struct
* args   : int    stop      I   stop position in function 0: run all
* return : none
*-----------------------------------------------------------------------------*/
extern void quitsdr(sdrini_t *ini, int stop)
{
    int i;
#ifdef GUI
    maindlg^form=static_cast<maindlg^>(hform.Target);
    form->guistop();
#endif
    if (stop==1) return;

    /* sdr termination */
    rcvquit(ini);
    if (stop==2) return;

    /* free memory */
    for (i=0;i<ini->nch;i++) freesdrch(&sdrch[i]);
    if (stop==3) return;

    /* mutexes and events */
    closehandles();
    if (stop==4) return;
}

void restart_acquisition(sdrch_t *sdr)
{
	sdr->trk.track_loss_cnt = 0;
	sdr->flagacq = 0;
	sdr->flagacq = 0;
	sdr->nav.flagdec = 0;
	sdr->nav.flagsync = 0;
	sdr->nav.flagsyncf = 0;
	sdr->nav.flagtow = 0;
	sdr->nav.synci = 0;
	sdr->nav.biti = 0;
	sdr->nav.swsync = 0;
	sdr->nav.swloop = 0;
	sdr->nav.bitIP = 0;
	sdr->nav.cnt = 0;
	sdr->nav.flagpol = 0;

	memset(sdr->nav.bitsync, 0, sdr->nav.rate * sizeof(int));

	clearcumsumcorr(&sdr->trk);

	sdr->trk.remcode = 0;
	sdr->trk.remcarr = 0;
	sdr->trk.oldremcarr = 0;
	sdr->trk.oldremcode = 0;
	sdr->trk.codeNco = 0;

	memset(sdr->trk.oldI, 0, 1 + 2 * sdr->trk.corrn * sizeof(double));
	memset(sdr->trk.oldQ, 0, 1 + 2 * sdr->trk.corrn * sizeof(double));

	sdr->nav.ocodei = 0;
}

/* sdr channel thread ----------------------------------------------------------
* sdr channel thread for signal acquisition and tracking . Every sat have its own thread
* args   : void   *arg      I   sdr channel struct
* return : none
* note : This thread handles the acquisition and tracking of one of the signals. 
*        The thread is created at startsdr function.
*-----------------------------------------------------------------------------*/
#ifdef WIN32
extern void sdrthread(void *arg)
#else
extern void *sdrthread(void *arg)
#endif
{
	using namespace std::chrono;

    sdrch_t *sdr=(sdrch_t*)arg;
    sdrplt_t pltacq={0},plttrk={0};
	/* Time is ms? */ 
	uint64_t cnt = 0;
    uint64_t buffloc=0,bufflocnow=0,loopcnt=0;
    double *acqpower=NULL;
    FILE* fp=NULL;
    char fname[100];
    
    /* create tracking log file */
    if (sdrini.log) {
        sprintf(fname,"log%s.csv",sdr->satstr);
        if((fp=createlog(fname,&sdr->trk))==NULL) {
            SDRPRINTF("error: invailed log file: %s\n",fname);
            return THRETVAL;
        }
    }

    /* plot setting */
    if (initpltstruct(&pltacq,&plttrk,sdr)<0) {
        sdrstat.stopflag=ON;
    }
    //sleepms(sdr->no*500);
    SDRPRINTF("**** %s sdr thread %d start! ****\n",sdr->satstr,sdr->no);

	long long pvev_seconds = 0;
	int prev_cnt = 0;

    while (!sdrstat.stopflag) 
	{
        /* acquisition */
        if (!sdr->flagacq) //if set, never reset
		{
            /* memory allocation */
            if (acqpower!=NULL) free(acqpower);
            acqpower=(double*)calloc(sizeof(double),sdr->nsamp*sdr->acq.nfreq);

            /* fft correlation. One of results is moving buffer position (buffloc)*/
            buffloc=sdraccuisition(sdr,acqpower);

            /* plot aquisition result */
            if (sdr->flagacq&&sdrini.pltacq) 
			{
                pltacq.z=acqpower;
                plot(&pltacq); //plot aquisition
            }
        }
        /* tracking - 1 GNSS code (1023 chips for GPS) */
        if (sdr->flagacq) 
		{
            bufflocnow=sdrtracking(sdr,buffloc,cnt); //correlator + sdrnavigation there
            if (sdr->flagtrk) //this flag is set periodicaly, after completed corellation
			{
                /* correlation output accumulation */
                cumsumcorr(&sdr->trk,sdr->nav.ocode[sdr->nav.ocodei]);

                sdr->trk.flagloopfilter=0;
                if (!sdr->nav.flagsync)
				{
					//Using Parameters1
                    pll(sdr,&sdr->trk.prm1,sdr->ctime); /* PLL */
                    dll(sdr,&sdr->trk.prm1,sdr->ctime); /* DLL */
                    sdr->trk.flagloopfilter=1;
                }
                else if (sdr->nav.swloop) 
				{
					//Using Parameters2
                    pll(sdr,&sdr->trk.prm2,(double)sdr->trk.loopms/1000);
                    dll(sdr,&sdr->trk.prm2,(double)sdr->trk.loopms/1000);
                    sdr->trk.flagloopfilter=2;

                    mlock(hobsmtx);

                    /* calculate observation data */
					if (loopcnt % (SNSMOOTHMS / sdr->trk.loopms) == 0)
					{
						setobsdata(sdr, buffloc, cnt, &sdr->trk, 1);
						//Detect tracking loss
						double summ_value = sdr->trk.Isum_fin / 1000.0;
						if (summ_value < TRACK_LOST_SUMM)
						{
							sdr->trk.track_loss_cnt++;
							if ((sdr->trk.track_loss_cnt > 
								(TRACK_RESTORE_TIME_MS / SNSMOOTHMS)) && (sdrini.use_restore_acq != 0))
							{
								restart_acquisition(sdr);
								cnt = 0;
							}
						}
						else
						{
							sdr->trk.track_loss_cnt = 0;
						}
					}
                    else
                        setobsdata(sdr,buffloc,cnt,&sdr->trk,0);

                    unmlock(hobsmtx);

                    /* plot correator output */
                    if (loopcnt%((int)(plttrk.pltms/sdr->trk.loopms))==0&&
                        sdrini.plttrk&&loopcnt>0) 
					{
                        plttrk.x=sdr->trk.corrx;
                        memcpy(plttrk.y,sdr->trk.sumI, sizeof(double)*(sdr->trk.corrn*2+1));
						//draw ABS values with a scale!
                        plotthread(&plttrk); //plot tracking
                    }
                    
                    /* LEX thread */
                    if (sdrini.nchL6!=0&&sdr->no==sdrini.nch+1&&loopcnt>250) 
                        setevent(hlexeve);

                    loopcnt++;
                }

                if (sdr->no==1 && (cnt%(1000*10)==0)) //10s when cnt is in ms
                    SDRPRINTF("Tracking process %d sec...\n",(int)cnt/(1000));
				
				// Display Tracking cycles diff.
				if ((sdr->no == 1) && (sdrini.dispay_track_cycles != 0))
				{
					high_resolution_clock::time_point t1 = high_resolution_clock::now();
					auto nanosec = t1.time_since_epoch();
					auto seconds = std::chrono::duration_cast<std::chrono::seconds>(nanosec);
					long long sec = seconds.count();

					if (pvev_seconds != sec)
					{
						int diff = cnt - prev_cnt;
						SDRPRINTF("Tracking cycles diff: %li\n", (int)diff);
						pvev_seconds = sec;
						prev_cnt = cnt;
					}
				}
				
				

                /* write tracking log */
				if (sdrini.log)
				{
					writelog(fp, &sdr->trk, &sdr->nav);
					//SDRPRINTF("LOG write\n");
				}

                if (sdr->trk.flagloopfilter) 
					clearcumsumcorr(&sdr->trk);
                cnt++;
				sdr->trk.track_cnt = cnt;
                buffloc+=sdr->currnsamp;
            }
        }
        sdr->trk.buffloc=buffloc;
    }//end of while
    
    if (sdrini.nchL6!=0&&sdr->no==sdrini.nch+1) 
        setevent(hlexeve);
    
    /* plot termination */
    quitpltstruct(&pltacq,&plttrk);

    /* close tracking log file */
	if (sdrini.log)
	{
		SDRPRINTF("LOG closed\n");
		closelog(fp);
	}

    if (sdr->flagacq) 
	{
        SDRPRINTF("SDR channel %s thread finished! Delay=%d [ms]\n",
            sdr->satstr,(int)(bufflocnow-buffloc)/sdr->nsamp);
    } 
	else
	{
        SDRPRINTF("SDR channel %s thread finished!\n",sdr->satstr);
    }

    return THRETVAL;
}
