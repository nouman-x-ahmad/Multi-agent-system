/*


NOUMAN AHMAD
ASJAD_RAUF
HARIS ALI MUGHAL



*/









#include <iostream>
#include <cmath>
#include <pthread.h>
#include <thread>
#include <semaphore.h>
#include <mutex>
#include <cstdlib>
#include <ctime>
#include<unistd.h>





using namespace std;

const int TTL_NUMB_OF_RBTS = 50;
const int TTL_NUMB_OF_THRDS_PR_RBTS= 50;
const int EST_T_SEC_ = 10;
const float exitt=50.0;
 mutex MTX_Locker;
float TTL_WIDTH = 0.0;
sem_t SEMO_LOCK;
float actual_wdth= 21.0;
const int SIZE_OF_ROOM = 100;

bool check =true;


//================================================================================================================================================


//robot structured data...;;;;
struct ST_ROBOT 
{
    float CORD_X;
    float CORD_Y;  // robot cordinates...;;;;
    float EST_EXIT_WDTH;  // exit_location...;;;;
    ST_ROBOT* NEIBOUR_RBTS_ARR[TTL_NUMB_OF_RBTS];  // data for neighboring bots...
};




//================================================================================================================================================














float CAL_RBT_DIST(const ST_ROBOT& RBT1, const ST_ROBOT& RBT2) ;
void SIMUL_RBT_BEH(ST_ROBOT& obj_bot) ;
void COMM_EST_BW_RBT(ST_ROBOT& obj_bot) ;
void* RBT_FUNC(void* args);
void CAL_N_AGGREG(ST_ROBOT obj_bots[]) ;
void print_map(const ST_ROBOT obj_bots[]) ;


// func to cal distance betwen two bots...;;;
float CAL_RBT_DIST(const ST_ROBOT& RBT1, const ST_ROBOT& RBT2) 
{
	
	
	float subs1=RBT1.CORD_X - RBT2.CORD_X;
	float subs2=RBT1.CORD_Y - RBT2.CORD_Y;
	float temp= hypot(subs1, subs2);
	
	
	
	
    return temp;
}




//================================================================================================================================================



float lenght_till_exit;

 float d=10.0;
        float f=5.0;




//simulate the robot..;;;;
void SIMUL_RBT_BEH(ST_ROBOT& obj_bot) 
{
    int t_iter = 0;
    
    
    while (t_iter < EST_T_SEC_) 
    {
    	++t_iter;
    	
    	

        lenght_till_exit = CAL_RBT_DIST(obj_bot, {exitt, exitt});  // exit data...;;;;

        float e = 1.0;
       
        
        
        
        if (d>=lenght_till_exit) 
        {
            float rnd=float(rand() % 5 + 1);
            e = 0.12 + 0.07 * rnd;
        }
        
        else if (f>=lenght_till_exit) 
        {
        	float rnd=float(rand() % 5 + 1);
            e = 0.05 * rnd;
            
        }
        
        else 
        {
        	float rnd=float(rand() % 5 + 1);
            e = 0.2 + 0.1 * rnd;
        }
	
	
	
        
        
        
        
        
        
        
        sem_wait(&SEMO_LOCK);
        float ext_wdth=actual_wdth * (1.0 + e);
        obj_bot.EST_EXIT_WDTH =ext_wdth;
        sem_post(&SEMO_LOCK);

    
    
        sleep(1);
    
    
    
    
    
    }
    
    
}






//================================================================================================================================================



float lngth_to_numb=0.0;
int denom=2.0;

//communication between bots...;;;;;;;
void COMM_EST_BW_RBT(ST_ROBOT& obj_bot) 
{
	int iter = 0;
    while ( iter < TTL_NUMB_OF_RBTS) 
    {
    	
        
        if (obj_bot.NEIBOUR_RBTS_ARR[iter]) 
        {
            lngth_to_numb = CAL_RBT_DIST(obj_bot, *obj_bot.NEIBOUR_RBTS_ARR[iter]);
    
    
            if (f>=lngth_to_numb) 
            {
                sem_wait(&SEMO_LOCK);
                float flx=(obj_bot.EST_EXIT_WDTH + obj_bot.NEIBOUR_RBTS_ARR[iter]->EST_EXIT_WDTH) / denom;
                obj_bot.EST_EXIT_WDTH = flx;
                sem_post(&SEMO_LOCK);
            }
            
            
            
        }
        
        
        
        
        
         ++iter;
        
        
    }
    
    
    
}




//================================================================================================================================================



void* RBT_FUNC(void* args)
{

    ST_ROBOT* robot = (ST_ROBOT*) args;



    SIMUL_RBT_BEH(*robot);
    COMM_EST_BW_RBT(*robot);




    return nullptr;



}










//================================================================================================================================================




int main() 
{
    sem_init(&SEMO_LOCK, 0, 1);

    ST_ROBOT obj_bots[TTL_NUMB_OF_RBTS];
   
   
   
    for (int i = 0; i < TTL_NUMB_OF_RBTS; ++i) 
    {
        
        
        obj_bots[i].CORD_X = float(rand() % 50);
        obj_bots[i].CORD_Y = float(rand() % 50);
        
        
        
        
        
        int loopy=0;
        for (int loopy = 0; loopy < TTL_NUMB_OF_RBTS; ++loopy)
        {
            obj_bots[i].NEIBOUR_RBTS_ARR[loopy] = nullptr;
        }
        
        
        
    }








    for (int i = 0; i < TTL_NUMB_OF_RBTS; ++i) 
    {
        int neighbor_count = 0;
        for (int j = 0; j < TTL_NUMB_OF_RBTS; ++j) 
        {
            if (i != j) 
            {
               
               
                float dist_bw_robot = 0;
                dist_bw_robot = CAL_RBT_DIST(obj_bots[i], obj_bots[j]);
               
               
                if (dist_bw_robot <= f) 
                {
                    obj_bots[i].NEIBOUR_RBTS_ARR[neighbor_count] = &obj_bots[j];
                    neighbor_count+=1;
                }
                
                
                
            }
            
            
        }
        
        
        
        
        
    }

    pthread_t numb_of_bot[TTL_NUMB_OF_RBTS * TTL_NUMB_OF_THRDS_PR_RBTS];
    
    
    for (int i = 0; i < TTL_NUMB_OF_RBTS; ++i) 
    {
        for (int jol = 0; jol < TTL_NUMB_OF_THRDS_PR_RBTS; ++jol) 
        {
            pthread_create(&numb_of_bot[i * TTL_NUMB_OF_THRDS_PR_RBTS+ jol], nullptr, RBT_FUNC, &obj_bots[i]);
        }
        
    }
    
    

     thread thread_for_calc(CAL_N_AGGREG, obj_bots);

    for (int t_iter = 0; t_iter < EST_T_SEC_; ++t_iter)
     {
        print_map(obj_bots);
         sleep(1);
    }
	
	
	
	int rbs_cnt=TTL_NUMB_OF_RBTS * TTL_NUMB_OF_THRDS_PR_RBTS;
    for (int iter = 0; iter < rbs_cnt; ++iter) 
    {
        pthread_join(numb_of_bot[iter], nullptr);
    }

    thread_for_calc.join();

    sem_destroy(&SEMO_LOCK);

    float average_width = TTL_WIDTH / TTL_NUMB_OF_RBTS;

     cout << "True exit width: 21.0";
     cout <<  endl;
     cout << "Average estimated width: ";
     cout << average_width;
     cout <<  endl;
     cout << "Difference: ";
     cout <<  abs(21.0 - average_width);
     cout <<  endl;

    return 0;
    
    
    
    
}
//================================================================================================================================================

















//================================================================================================================================================



void print_map(const ST_ROBOT obj_bots[]) 
{
     MTX_Locker.lock();
     
     
     
     
 
    char arr_room[SIZE_OF_ROOM][SIZE_OF_ROOM];
    int iter = 0;

    
    
    
    while ( iter < SIZE_OF_ROOM ) 
    {
    	int loop = 0;
        
        
        
        for (; loop < SIZE_OF_ROOM; ++loop) 
        {
            arr_room[iter][loop] = ' ';
        }
        
        
        
        
        ++iter;
        
    }





    for (int i = 0; i < TTL_NUMB_OF_RBTS; ++i) 
    {
        int CORD_X = int(obj_bots[i].CORD_X);
        int CORD_Y = int(obj_bots[i].CORD_Y);
        if (CORD_X >= 0 && SIZE_OF_ROOM > CORD_Y &&  SIZE_OF_ROOM > CORD_X && CORD_Y >= 0 ) 
        {
            arr_room[CORD_Y][CORD_X] = 'R';
        }
        
    }
	if(check==true)
    	{
    		cout << "ROOM SIMULATION";
     		cout <<  endl;
    	}
     
     
     
     
     
    for (int loopi = 0; loopi < SIZE_OF_ROOM; ++loopi) 
    {
    	if(check==false)
    	{
    		break;
    	}
        for (int loopj = 0; loopj < SIZE_OF_ROOM; ++loopj) 
        {
        
            if(!(loopi == 50 && loopj == 50))
            {
                 cout << arr_room[loopi][loopj];
            }
        
        
            else if (loopi == 50 && loopj == 50) 
            {
                 cout << 'E';
            }
            
            
            
        }
         cout <<  endl;
        
    }
    
    
    
    
    
    
    
    
     MTX_Locker.unlock();
      check =false;
      
      
      
      
}




//================================================================================================================================================




void CAL_N_AGGREG(ST_ROBOT obj_bots[]) 
{
     sleep(1);
	int iter = 0; 
    while (iter < TTL_NUMB_OF_RBTS ) 
    {
        sem_wait(&SEMO_LOCK);
        TTL_WIDTH += obj_bots[iter].EST_EXIT_WDTH;
        sem_post(&SEMO_LOCK);
        
        ++iter;
    }
    
}








