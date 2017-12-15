#include <pthread.h>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/posetracker.h>
#include <stdio.h>  
#include <string.h>
#include <math.h>  
#include <string.h>
#include <unistd.h>
#include "highgui.h"  
#include <cv.h>  
#include <opencv2/opencv.hpp>  
#include <opencv2/highgui/highgui.hpp>  

using namespace cv;
using namespace aruco;

//global variables
float Rotat_Vec_Arr[3];//rotational vector array
float Rotat_M[9];//rotational matrix array
float Trans_M[3];//translation vector array

pthread_mutex_t IK_Solver_Lock;

/*******************************************************************************************************************
function declaration
********************************************************************************************************************/
void *Thread_Func_Message_Send(void *arg);//thread_function
void InitMat(Mat& m,float* num);
void PrintMat(CvMat *matrix, bool save_or_show, FILE *fp);

/*******************************************************************************************************************
main function
********************************************************************************************************************/
int main(int argc,char **argv)
{

    int thread_return;
    pthread_t Message_Send_Thread_ID;
    //init thread lock
    pthread_mutex_init(&IK_Solver_Lock, NULL);
    //creat new thread 
    thread_return = pthread_create(&Message_Send_Thread_ID,NULL,Thread_Func_Message_Send,NULL);
    
    //import the camera param (CameraMatrix)
    float camera_matrix_array[9] = { 1.0078520005023535e+003, 0., 6.3950000000000000e+002, 
                                  0.0, 1.0078520005023535e+003, 3.5950000000000000e+002, 
                                  0.0, 0.0, 1.0 };
    cv::Mat Camera_Matrix(3,3,CV_32FC1);
    InitMat(Camera_Matrix,camera_matrix_array);
    cout << "Camera_Matrix = " << endl << "" << Camera_Matrix << endl ;
    //import the camera param (Distorsion)
    float Distorsion_array[5] = {-4.9694653328469340e-002, 2.3886698343464000e-001, 0., 0.,-2.1783942538569392e-001};
    cv::Mat Distorsion_M(1,5,CV_32FC1);
    InitMat(Distorsion_M,Distorsion_array);
    cout << "Distorsion_M = " << endl << "" << Distorsion_M << endl ;

    CameraParameters LogiC170Param;
    //LogiC170Param.readFromXMLFile("LogitchC170_Param.yml");
    LogiC170Param.CameraMatrix = Camera_Matrix.clone();
    LogiC170Param.Distorsion = Distorsion_M.clone();
    LogiC170Param.CamSize.width = 1280;
    LogiC170Param.CamSize.height = 720;
    //cout << "LogiC170Param.CameraMatrix = " << endl << "" << LogiC170Param.CameraMatrix << endl ;
    //cout << "LogiC170Param.Distorsion = " << endl << "" << LogiC170Param.Distorsion << endl ;

    float MarkerSize = 0.04;
    int Marker_ID;
    MarkerDetector MDetector;
    MDetector.setThresholdParams(7, 7);
    MDetector.setThresholdParamRange(2, 0);

    CvDrawingUtils MDraw;

    //read the input image
    VideoCapture cap(0); // open the default camera 
     if(!cap.isOpened())  // check if we succeeded  
        return -1; 
    cv::Mat frame;
    cv::Mat Rvec;//rotational vector
    CvMat Rvec_Matrix;//temp matrix
    CvMat R_Matrix;//rotational matrixs
    cv::Mat Tvec;//translation vector

    cap>>frame;//get first frame
    //LogiC170Param.resize(frame.size());

    printf("%f, %f\n",cap.get(CV_CAP_PROP_FRAME_WIDTH),cap.get(CV_CAP_PROP_FRAME_HEIGHT));  
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);  
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);  
    //cap.set(CV_CAP_PROP_FPS, 10);  
    printf("%f, %f\n",cap.get(CV_CAP_PROP_FRAME_WIDTH),cap.get(CV_CAP_PROP_FRAME_HEIGHT));   

    while(1)
    {
        //get current frame
        cap>>frame;
        //Ok, let's detect
        vector< Marker >  Markers=MDetector.detect(frame, LogiC170Param, MarkerSize);
        //printf("marker count:%d \n",(int)(Markers.size()));

        //for each marker, estimate its ID and if it is  100 draw info and its boundaries in the image
        for (unsigned int j=0;j<Markers.size();j++)
        {
            
            //marker ID test
            Marker_ID = Markers[j].id;
            printf("Marker ID = %d \n",Marker_ID);

            if(Marker_ID == 100)
            {
                //cout<<Markers[j]<<endl;
                Markers[j].draw(frame,Scalar(0,0,255),2);

                Markers[j].calculateExtrinsics(MarkerSize, LogiC170Param, false);
                //calculate rotational vector
                Rvec = Markers[j].Rvec;
                cout << "Rvec = " << endl << "" << Rvec << endl ;
                //calculate transformation vector
                Tvec = Markers[j].Tvec;
                cout << "Tvec = " << endl << "" << Tvec << endl ;

                //lock to update global variables: Rotat_Vec_Arr[3]  Rotat_M[9]  Trans_M[3]
                pthread_mutex_lock(&IK_Solver_Lock);

                //save rotational vector to float array
                for (int r = 0; r < Rvec.rows; r++)  
                {  
                    for (int c = 0; c < Rvec.cols; c++)  
                    {     
                        //cout<< Rvec.at<float>(r,c)<<endl;  
                        Rotat_Vec_Arr[r] = Rvec.at<float>(r,c);
                    }     
                }
                printf("Rotat_Vec_Arr[3] = [%f, %f, %f] \n",Rotat_Vec_Arr[0],Rotat_Vec_Arr[1],Rotat_Vec_Arr[2]);

                //save array data to CvMat and convert rotational vector to rotational matrix
                cvInitMatHeader(&Rvec_Matrix,1,3,CV_32FC1,Rotat_Vec_Arr,CV_AUTOSTEP);//init Rvec_Matrix
                cvInitMatHeader(&R_Matrix,3,3,CV_32FC1,Rotat_M,CV_AUTOSTEP);//init R_Matrix and Rotat_M
                cvRodrigues2(&Rvec_Matrix, &R_Matrix,0);
                printf("Rotat_M = \n[%f, %f, %f, \n  %f, %f, %f, \n  %f, %f, %f] \n",Rotat_M[0],Rotat_M[1],Rotat_M[2],Rotat_M[3],Rotat_M[4],Rotat_M[5],Rotat_M[6],Rotat_M[7],Rotat_M[8]);
                
                //save transformation vector to float array
                for (int r = 0; r < Tvec.rows; r++)
                {  
                    for (int c = 0; c < Tvec.cols; c++)  
                    {
                        Trans_M[r] = Tvec.at<float>(r,c);
                    }
                }
                printf("Trans_M[3] = [%f, %f, %f] \n",Trans_M[0],Trans_M[1],Trans_M[2]);

                //unlock 
                pthread_mutex_unlock(&IK_Solver_Lock);

                // draw a 3d cube in each marker if there is 3d info
                if (LogiC170Param.isValid() && MarkerSize != -1)
                {
                    MDraw.draw3dAxis(frame,LogiC170Param,Rvec,Tvec,0.04);
                }

            }

        }

        //*/
        cv::waitKey(150);//wait for key to be pressed
        cv::imshow("Frame",frame);

    }

    //wait for the IK solver thread close and recover resources
    pthread_join(Message_Send_Thread_ID,NULL);

    pthread_mutex_destroy(&IK_Solver_Lock); //destroy the thread lock
    return 0;

}

/**********************************************************
function: new thread to send messages 
input: void
return :null
***********************************************************/
void * Thread_Func_Message_Send(void *arg)
{
    printf("IK solver thread is running!\n");
    //original pose and position
    float P_original[4];
    float N_original[4];
    float O_original[4];
    float A_original[4];
    //final pose and position 
    float P[3];
    float N[3];
    float O[3];
    float A[3];

    P_original[3] = 1;
    N_original[3] = 0;
    O_original[3] = 0;
    A_original[3] = 0;

    while (1)
    {
        //get the spacial pose
        pthread_mutex_lock(&IK_Solver_Lock);
        //memcpy(P_original, Trans_M, sizeof(Trans_M));
        for(int i=0;i<3;i++)
        {
            P_original[i] = Trans_M[i];
            N_original[i] = Rotat_M[3*i];
            O_original[i] = Rotat_M[3*i+1];
            A_original[i] = Rotat_M[3*i+2];
        }
        pthread_mutex_unlock(&IK_Solver_Lock);
        //debug printf
        ///*
        printf("N_original[4] = [%f, %f, %f, %f]  \n",N_original[0],N_original[1],N_original[2],N_original[3]);
        printf("O_original[4] = [%f, %f, %f, %f]  \n",O_original[0],O_original[1],O_original[2],O_original[3]);
        printf("A_original[4] = [%f, %f, %f, %f]  \n",A_original[0],A_original[1],A_original[2],A_original[3]);
        printf("P_original[4] = [%f, %f, %f, %f]  \n",P_original[0],P_original[1],P_original[2],P_original[3]);
        //*/

        printf("I send the message to robot here! \n");
        /*
        add message send function here!
        */

        //uodate every 5 s
        sleep(5);
    }

    //kill the message send thread
    pthread_exit(0);
        
}

/**********************************************************
function: copy arrar element to CV::Mat
input: CV::Mat  and array point
return :null
***********************************************************/
void InitMat(Mat& m,float* num)
{
 for(int i=0;i<m.rows;i++)
  for(int j=0;j<m.cols;j++)
   m.at<float>(i,j)=*(num+i*m.rows+j);
}

/********************************* 
function  PrintMat(CvMat *matrix) (print the cvMat on screen) 
input：matrix point
return：null
**********************************/  
void PrintMat(CvMat *matrix, bool save_or_show =false,FILE *fp=NULL)  
{  
    int i=0;  
    int j=0;  
    for(i=0;i < matrix->rows;i++)//行  
    {  
        if (save_or_show)  
        {  
            fprintf(fp,"\n");  
        }   
        else  
        {  
            printf("\n");  
        }  
        switch(matrix->type&0X07)  
        {  
        case CV_32F:   
        case CV_64F:  
            {  
                for(j=0;j<matrix->cols;j++)//列  
                {  
                    if (save_or_show)  
                    {  
                        fprintf(fp,"%9.3f ",(float)cvGetReal2D(matrix,i,j));  
                    }   
                    else  
                    {  
                        printf("%9.3f ",(float)cvGetReal2D(matrix,i,j));  
                    }  
                }  
                break;  
            }  
        case CV_8U:  
        case CV_16U:  
            {  
                for(j=0;j<matrix->cols;j++)  
                {  
                    printf("%6d  ",(int)cvGetReal2D(matrix,i,j));  
                    if (save_or_show)  
                    {  
                        fprintf(fp,"%6d  ",(int)cvGetReal2D(matrix,i,j));  
                    }   
                    else  
                    {  
                        printf("%6d  ",(int)cvGetReal2D(matrix,i,j));  
                    }  
                }  
                break;  
            }  
        default:  
            break;  
        }  
    }  
}  
//*****************************


