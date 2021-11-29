#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <unistd.h>
#include <thread>
#include <opencv2/core/types.hpp>

namespace NUC606
{
	// ��ά�µ�Ԥ��
	class StatePredict
	{
	public:
		StatePredict() = default;     /*���캯��*/

		StatePredict(cv::Mat A, cv::Mat H, cv::Mat R, cv::Mat Q, cv::Mat P, cv::Mat x_k1);

		~StatePredict() {}  // ��������

		void InitFilter(cv::Point3f a_position); // ��ʼ���˲���

		void ReadParam(); // ��ȡһЩ����

		cv::Point3f Predict(cv::Point3f); 

	private:

		std::shared_ptr<cv::KalmanFilter> m_KF;

		int m_freq; 
		bool m_init = false;
		int m_init_count_max;
		float m_dt;  // ״̬ת���������ʱ�����
		int m_measure_num = 3; //�۲������Ԫ�ظ��� 
		int m_state_num = 9;  //״̬������Ԫ�ظ��� ÿһ�������λ��p���ٶ�v�����ٶ�a ��3*3=9
		

		/*****************************************************/
		/*��Ҫ��ȡ�Ĳ���*/
		int init_count_threshold;  //���ڵ�һ֡��Ԥ��Ĵ��� �����Ԥ��ȡ����ֵ��
		float cur_armor_dis;  // ��ǰװ�װ�ľ���
		float last_armor_dis; // ��һ��װ�װ�ľ���
		int m_debug;
		cv::Mat R_CI_MAT;               // ����������ϵ���������ϵ��ת����CV-Mat
		cv::Mat F_MAT;                  // ����ڲξ���CV-Mat
		cv::Mat C_MAT;                  // ����������CV-Mat
		/*****************************************************/

		cv::Mat measurement;    // �۲����
		cv::Mat x_k;            // x_kʱ�̵�ֵ������ֵ  ����ʱ��
		cv::Point3f x_k1;			// x_k-1ʱ�̵�ֵ������ֵ  ��һʱ��
		cv::Mat A;			  // A ״̬ת������
		cv::Mat H;			 // H �۲����
		cv::Mat Q;			// Q Ԥ������
		cv::Mat R;			// R �۲�����
		cv::Mat P;        // P ϵͳЭ����
	

		
	};

}