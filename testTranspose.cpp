#include <ctime>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <armadillo>
#include <immintrin.h>

#include <boost/align/aligned_allocator.hpp>

#define OFDM 1024
#define BS_ANT 96
#define K 4

#define LOOP_NUM 1e3


struct complex_float {
    float real;
    float imag;
};

//typedef std::vector<complex_float> myVec;
typedef std::vector<complex_float, boost::alignment::aligned_allocator<complex_float, 128>> myVec;
using namespace std;
using namespace arma;
typedef cx_float COMPLEX;

int flushCache()
{
	const size_t bigger_than_cachesize = 100 * 1024 * 1024;
	long *p = new long[bigger_than_cachesize];
	// When you want to "flush" cache. 
	for(int i = 0; i < bigger_than_cachesize; i++)
	{
	   p[i] = rand();
	}
	delete p;
}

void saveData(char* filename, complex_float* ptr, int row, int col)
{
	FILE* fp = fopen(filename, "w");
	for(int i = 0; i < row; i++)
	{
		for(int j = 0; j < col; j++)
		{
			fprintf(fp, "%6.5f+%6.5fi  ", ptr[i * col + j].real, ptr[i * col + j].imag);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}


int main(int argc, char** argv)
{
				//__m128i index = _mm_setr_epi32(0, 1, 2, 3);
				__m256i index = _mm256_setr_epi64x(0, 4, 8, 12);
			
				myVec debug_memory;
				debug_memory.resize(16);
				float* debug_data_float = (float *)debug_memory.data();
				for(int kk = 0; kk < 32; kk++)
					debug_data_float[kk] = 1.0f * kk;
				__m256d t_data = _mm256_i64gather_pd((double*)debug_data_float, index, 8);
				//__m256d t_data = _mm256_load_pd((double*)debug_data_float);

				// debug
				double debug[4];
				_mm256_store_pd(debug, t_data);
				float* debug_float_ptr = (float*)debug;
				printf("%f %f %f %f %f %f %f %f\n", debug_float_ptr[0], debug_float_ptr[1], debug_float_ptr[2], debug_float_ptr[3],
					debug_float_ptr[4], debug_float_ptr[5], debug_float_ptr[6], debug_float_ptr[7]);

	srand(0);
	printf("test\n");
	myVec buffer;
	myVec buffer_trans;
	buffer.resize(BS_ANT * OFDM);
	buffer_trans.resize(BS_ANT * OFDM);
	for(int i = 0; i < BS_ANT; i++)
	{
		for(int j = 0; j < OFDM; j++)
		{
			buffer[i * OFDM + j].real = (rand()%65536) / (float)65536;
			buffer[i * OFDM + j].imag = (rand()%65536) / (float)65536;
			//buffer[i * OFDM + j].real = 1.f;
			//buffer[i * OFDM + j].imag = 0.f;
		}
	}	

	saveData("data.txt", buffer.data(), BS_ANT, OFDM);

	myVec precoder;
	precoder.resize(K * BS_ANT);
	for(int i = 0; i < K * BS_ANT; i++)
	{
		precoder[i].real = (rand()%65536) / (float)65536;
		precoder[i].imag = (rand()%65536) / (float)65536;
		//precoder[i].real = 1.f;
		//precoder[i].imag = 0.f;
	}
	myVec result;
	result.resize(K);

	saveData("precoder.txt", precoder.data(), K, BS_ANT);


	flushCache();
	auto begin = std::chrono::system_clock::now();
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		// just copy
		for(int j = 0; j < BS_ANT; j++)
		{
			//memcpy(buffer_trans.data() + j * OFDM, buffer.data() + j * OFDM, sizeof(complex_float) * OFDM);
			std::copy(buffer.data() + j * OFDM, buffer.data() + (j + 1) * OFDM, buffer_trans.data() + j * OFDM);
		}
	}
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> diff = end - begin;
	printf("memcpy copy time %f\n", diff.count());

	flushCache();
	begin = std::chrono::system_clock::now();
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		// just copy
		for(int c1 = 0; c1 < BS_ANT; c1++)
		{
			for(int c2 = 0; c2 < OFDM; c2++)
			{
				buffer_trans[c1 * OFDM + c2] = buffer[c1 * OFDM + c2];
			}
		}
	}
	end = std::chrono::system_clock::now();
	diff = end - begin;
	printf("naive copy time %f\n", diff.count());

	flushCache();
	begin = std::chrono::system_clock::now();
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		float* src_ptr = (float*)buffer.data();
		float* tar_ptr = (float*)buffer_trans.data();
		for(int i = 0; i < BS_ANT * OFDM / 4; i++)
		{
			__m256 data = _mm256_load_ps(src_ptr);
			_mm256_store_ps(tar_ptr, data);
			src_ptr += 8;
			tar_ptr += 8;
		}
	}
	end = std::chrono::system_clock::now();
	diff = end - begin;
	printf("avx2 __m256 copy time %f\n", diff.count());

	
	flushCache();
	begin = std::chrono::system_clock::now();
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		// just copy
		for(int c1 = 0; c1 < BS_ANT; c1++)
		{
			for(int c2 = 0; c2 < OFDM; c2++)
			{
				buffer_trans[c2 * BS_ANT + c1] = buffer[c1 * OFDM + c2];
			}
		}
	}
	end = std::chrono::system_clock::now();
	diff = end - begin;
	printf("naive trans time %f\n", diff.count());


		
	flushCache();
	begin = std::chrono::system_clock::now();
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		// just copy
		for(int j = 0; j < BS_ANT; j++)
		{
			memcpy(buffer_trans.data() + j * OFDM, buffer.data() + j * OFDM, sizeof(complex_float) * OFDM);
		}
		cx_float* mat_ptr = (cx_float *)buffer_trans.data();
        cx_fmat mat_data(mat_ptr, BS_ANT, OFDM, false);
        inplace_trans(mat_data);
	}
	end = std::chrono::system_clock::now();
	diff = end - begin;
	printf("armadillo trans time %f\n", diff.count());


	flushCache();
	begin = std::chrono::system_clock::now();
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		for(int c1 = 0; c1 < OFDM; c1++)
		{
			cx_float* data_ptr = (cx_float *)(&buffer[c1 * BS_ANT]);
			cx_fmat mat_data(data_ptr, BS_ANT, 1, false);

			cx_float* precoder_ptr = (cx_float*)precoder.data();
			cx_fmat mat_precoder(precoder_ptr, K, BS_ANT, false);

			cx_float* result_ptr = (cx_float*)result.data();
			cx_fmat mat_result(result_ptr, K, 1, false);

			mat_result = mat_precoder * mat_data;
			
		}
	}
	end = std::chrono::system_clock::now();
	diff = end - begin;
	printf("only precoding time %f\n", diff.count());



	flushCache();
	begin = std::chrono::system_clock::now();
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		// just copy
		for(int c1 = 0; c1 < BS_ANT; c1++)
		{
			for(int c2 = 0; c2 < OFDM; c2++)
			{
				buffer_trans[c2 * BS_ANT + c1] = buffer[c1 * OFDM + c2];
			}
		}
		for(int c1 = 0; c1 < OFDM; c1++)
		{
			cx_float* data_ptr = (cx_float *)(&buffer_trans[c1 * BS_ANT]);
			cx_fmat mat_data(data_ptr, BS_ANT, 1, false);

			cx_float* precoder_ptr = (cx_float*)precoder.data();
			cx_fmat mat_precoder(precoder_ptr, K, BS_ANT, false);

			cx_float* result_ptr = (cx_float*)result.data();
			cx_fmat mat_result(result_ptr, K, 1, false);

			mat_result = mat_precoder * mat_data;

			if(i == 0 && c1 == 0)
			{
				saveData("demul_ca_0_baseline.txt", (complex_float*)result_ptr, K, 1);
				saveData("data_ca_0_baseline.txt", (complex_float*)data_ptr, BS_ANT, 1);
				saveData("precoder_ca_0_baseline.txt", (complex_float*)precoder_ptr, K, BS_ANT);
			}
		}
	}
	end = std::chrono::system_clock::now();
	diff = end - begin;
	printf("naive trans and precoding time %f\n", diff.count());

	saveData("data_trans.txt", buffer_trans.data(), OFDM, BS_ANT);


	flushCache();
	begin = std::chrono::system_clock::now();
	myVec temp_buffer;
	temp_buffer.resize(BS_ANT);
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		for(int c1 = 0; c1 < OFDM; c1++)
		{
			for(int c2 = 0; c2 < BS_ANT; c2++)
				temp_buffer[c2] = buffer[c2 * OFDM + c1];

			cx_float* data_ptr = (cx_float *)(&temp_buffer[0]);
			cx_fmat mat_data(data_ptr, BS_ANT, 1, false);

			cx_float* precoder_ptr = (cx_float*)precoder.data();
			cx_fmat mat_precoder(precoder_ptr, K, BS_ANT, false);

			cx_float* result_ptr = (cx_float*)result.data();
			cx_fmat mat_result(result_ptr, K, 1, false);

			mat_result = mat_precoder * mat_data;

			if(i == 0 && c1 == 0)
			{
				saveData("demul_ca_0_read.txt", (complex_float*)result_ptr, K, 1);
				saveData("data_ca_0_read.txt", (complex_float*)data_ptr, BS_ANT, 1);
				saveData("precoder_ca_0_read.txt", (complex_float*)precoder_ptr, K, BS_ANT);
			}
		}
	}
	end = std::chrono::system_clock::now();
	diff = end - begin;
	printf("no copy and (read trans) precoding time %f\n", diff.count());

/*
	begin = std::chrono::system_clock::now();
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		// save buffer_trans as 8 column blocks
		float* src_ptr = (float*)buffer.data();
		float* tar_ptr = (float*)buffer_trans.data();
		for(int c1 = 0; c1 < BS_ANT; c1++)
		{
			for(int c2 = 0; c2 < OFDM / 4; c2++)
			{
				__m256 data = _mm256_load_ps(src_ptr + c1 * OFDM * 2 + c2 * 8);
				_mm256_store_ps(tar_ptr + c2 * BS_ANT * 8 + 8 * c1, data);
			}
		}

	}
	end = std::chrono::system_clock::now();
	diff = end - begin;
	printf("block trans time %f\n", diff.count());
*/

	flushCache();
	begin = std::chrono::system_clock::now();
	float* temp_buffer_ptr = (float*)(temp_buffer.data());
	for (int i = 0; i < LOOP_NUM; ++i)
	{
		// save buffer_trans as 8 column blocks
		float* src_ptr = (float*)buffer.data();
		float* tar_ptr = (float*)buffer_trans.data();
		for(int c1 = 0; c1 < BS_ANT; c1++)
		{
			for(int c2 = 0; c2 < OFDM / 4; c2++)
			{
				__m256 data = _mm256_load_ps(src_ptr + c1 * OFDM * 2 + c2 * 8);
				_mm256_store_ps(tar_ptr + c2 * BS_ANT * 8 + 8 * c1, data);
			}
		}


		__m256i index = _mm256_setr_epi64x(0, 4, 8, 12);
		for(int c1 = 0; c1 < OFDM; c1++)
		{
			for(int c2 = 0; c2 < BS_ANT / 4; c2++)
			{
				
				int c1_base = c1 / 4;
				int c1_offset = c1 % 4;
				float* base_ptr = tar_ptr + c1_base * 8 * BS_ANT + c1_offset * 2 + c2 * 4 * 8;
				__m256d t_data = _mm256_i64gather_pd((double*)base_ptr, index, 8);
				_mm256_store_pd((double*)(temp_buffer_ptr + c2 * 8), t_data);
				

				//__m256d t_data = _mm256_i64gather_pd((double*)tar_ptr, index, 8);
				//_mm256_store_pd((double*)temp_buffer_ptr, t_data);

				if(c1 == 0 && i == 0 && c2 == 0)
					for(int kk = 0; kk < 8; kk++)
						printf("%f\n", temp_buffer_ptr[kk]);
			}
			

			cx_float* data_ptr = (cx_float *)(&temp_buffer[0]);
			cx_fmat mat_data(data_ptr, BS_ANT, 1, false);

			cx_float* precoder_ptr = (cx_float*)precoder.data();
			cx_fmat mat_precoder(precoder_ptr, K, BS_ANT, false);

			cx_float* result_ptr = (cx_float*)result.data();
			cx_fmat mat_result(result_ptr, K, 1, false);

			mat_result = mat_precoder * mat_data;

			if(i == 0 && c1 == 0)
			{
				saveData("demul_ca_0_SIMD.txt", (complex_float*)result_ptr, K, 1);
				saveData("data_ca_0_SIMD.txt", (complex_float*)data_ptr, BS_ANT, 1);
				saveData("precoder_ca_0_SIMD.txt", (complex_float*)precoder_ptr, K, BS_ANT);
			}
		}

	}
	end = std::chrono::system_clock::now();
	diff = end - begin;
	printf("no copy and (SIMD read trans) precoding time %f\n", diff.count());
	saveData("data_trans_block.txt", buffer_trans.data(), OFDM*BS_ANT / 4, 4);
	
}
