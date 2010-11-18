/*******************************************************************************
 *
 *      ann_parser
 * 
 *      Copyright (c) 2010, edward
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "seabee3-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef ANN_PARSER_H_
#define ANN_PARSER_H_

#include <libvml/artificial_neural_network.h>
#include <sstream>
#include <stdio.h>
#include <string>

namespace vml
{

	class AnnParser
	{
	public:
		template<class _InputDataType, class _OutputDataType, class _InternalDataType>
		static bool readAnnFromFile( ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType> & ann, const char* path )
		{
			FILE * in = fopen( path, "r" );

			if ( !in ) return false;

			float learning_rate;
			fscanf( in, "%f", &learning_rate );

			int dims_size;
			fscanf( in, "%d", &dims_size );

			_Dimension dims( dims_size );

			for ( int i = 0; i < dims_size; i++ )
			{
				int dims_i;
				fscanf( in, "%d", &dims_i );

				dims[i] = dims_i;
			}

			//ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType> ann( dims, learning_rate );

			ann = ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType> ( dims, learning_rate );

			int output_scales_size;
			fscanf( in, "%d", &output_scales_size );

			ann.output_scales_.resize( output_scales_size );

			for ( int i = 0; i < output_scales_size; i++ )
			{
				float output_scales_i;
				fscanf( in, "%f", &output_scales_i );

				ann.output_scales_[i] = output_scales_i;
			}

			int weights_size;
			fscanf( in, "%d", &weights_size );

			for ( int i = 0; i < weights_size; i++ )
			{
				int weights_i_size;
				fscanf( in, "%d", &weights_i_size );


				//ann.weights_[i].resize( weights_i_size );

				for ( int j = 0; j < weights_i_size; j++ )
				{
					float weights_i_j;
					fscanf( in, "%f", &weights_i_j );

					ann.weights_[i][j] = weights_i_j;
				}
			}

			fclose( in );
			return true;
		}

		template<class _InputDataType, class _OutputDataType, class _InternalDataType>
		static bool saveAnnToFile( ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType> & ann, const char * path )
		{
			FILE * out = fopen( path, "w+" );

			if ( !out ) return false;

			fprintf( out, "%f ", ann.learning_rate_ );

			fprintf( out, "%d ", ann.layer_dims_.size() );
			for ( size_t i = 0; i < ann.layer_dims_.size(); i++ )
			{
				fprintf( out, "%d ", ann.layer_dims_[i] );
			}

			fprintf( out, "%d ", ann.output_scales_.size() );
			for ( size_t i = 0; i < ann.output_scales_.size(); i++ )
			{
				fprintf( out, "%f ", ann.output_scales_[i] );
			}

			fprintf( out, "%d ", ann.weights_.size() );
			for ( size_t i = 0; i < ann.weights_.size(); i++ )
			{
				fprintf( out, "%d ", ann.weights_[i].size() );
				for ( size_t j = 0; j < ann.weights_[i].size(); j++ )
				{
					fprintf( out, "%f ", ann.weights_[i][j] );
				}
			}

			/*fprintf( out, "%d ", ann.nodes_.size() );
			 for ( size_t i = 0; i < ann.nodes_.size(); i++ )
			 {
			 fprintf( out, "%d ", ann.nodes_[i].size() );
			 for ( size_t j = 0; j < ann.nodes_[i].size(); j++ )
			 {
			 fprintf( out, "%d %d %f ", ann.nodes_[i][j].type_, ann.nodes_[i][j].activation_function_.type_, ann.nodes_[i][j].activation_function_.coefficient_ );
			 }
			 }*/

			fclose( out );

			return true;
		}
	};

}

#endif /* ANN_PARSER_H_ */
