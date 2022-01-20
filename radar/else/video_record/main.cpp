#include "control/rm_link.h"
#include<ctime>
int main()
{
#if RECORD == 1
    RM_Vision_Init run;
    double sum=0;
    while(true)
    {
        //double t = double(getTickCount());
        clock_t start=std::clock();
        /** run **/
        run.Run();
        //t = (double(getTickCount() - t)) / getTickFrequency();
        clock_t end=std::clock();
        double endtime=(double)(end-start)/CLOCKS_PER_SEC;
        sum+=endtime;
        if(sum>=5){
            run.close();
            sum=sum-5;
        }
#if COUT_FPS == 1
        int fps = int(1.0 / t);
        cout<< endl << "FPS: " << fps<< endl;
#endif

        #if ANALYZE_EACH_FRAME == 1
        if(run.is_continue()){
            continue;
        }
        #endif

        if(run.is_exit()){
            break;
        }
    }

    destroyAllWindows();

    return EXIT_SUCCESS;
#endif
#if RECORD == 2
    RM_Vision_Init run;
    for (;;)
    {
        run.PHOTO_RUN();
        if(run.is_exit()){
            break;
        }
    }
    

#endif
}
