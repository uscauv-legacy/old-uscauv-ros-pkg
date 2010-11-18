/*******************************************************************************
 *
 *      artificial_neural_network
 * 
 *      Copyright (c) 2010,
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com)
 *
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

#ifndef ARTIFICIAL_NEURAL_NETWORK_H_
#define ARTIFICIAL_NEURAL_NETWORK_H_

#include <libvml/vector.h>
#include <stdio.h>
#include <time.h>
#include <sstream>

#include <libvml/node.h>

// model
// 2:4:3
//
//  |i
// -+---------
// j|0:0 | 0:0
//  |0:1 | 0:1
//  |0:2 | 0:2
//  |0:3 | 1:0
//  |1:0 | 1:1
//  |1:1 | 1:2
//  |1:2 | 2:0
//  |1:3 | 2:1
//  |    | 2:2
//  |    | 3:0
//  |    | 3:1
//  |    | 3:2
//
// weights needs to be 2*4x4*3 = 8x12
// the weight W_n_ij between the ith node in layer n and the jth node in layer n+1 is weights[n][i*max(j+1)+j]

namespace vml
{
	typedef Vector<uint> _Dimension;

	template<class _InputDataType = bool, class _OutputDataType = bool, class _InternalDataType = double>
	class ArtificialNeuralNetwork
	{

	public:
		typedef Vector<_InputDataType> _InputVector;
		typedef Vector<_OutputDataType> _OutputVector;
		typedef Vector<_InternalDataType> _InternalDataVector;

		// 2d matrix of weights
		Vector<_InternalDataVector> weights_;
		// 2d matrix of nodes
		Vector<Vector<Node<_InternalDataType> > > nodes_;
		// vector of layers
		_Dimension layer_dims_;
		_InternalDataType learning_rate_;

		// calculated automatically from training data
		// the goal is to scale the outputs to fit in [-1,1] for training
		_InternalDataVector output_scales_;

		ArtificialNeuralNetwork( _Dimension layer_dims = _Dimension(), _InternalDataType learning_rate = 0.5 );
		virtual ~ArtificialNeuralNetwork();
		void initializeRandomWeights( _InputDataType min = (_InternalDataType) -1.0, _InternalDataType max = (_InternalDataType) 1.0 );
		void resize( _Dimension & layer_dims );
		std::string toString();
		_InternalDataType & getWeight( int n, int i, int j );
		_OutputVector propagateData( _InputVector & input, bool scale_output = true );
		_InternalDataVector getErrorVector( _InputVector & input, _OutputVector & desired_output );
		_InternalDataType getTotalSystemError( Vector<_InternalDataVector> & error );
		void backpropagateErrorVector( _InternalDataVector & error );
		int train( std::vector<_InputVector> & inputs, std::vector<_OutputVector> & outputs, double min_error = 0.1, int max_iterations = 50000 );
		_InputVector getInputVector();
		_OutputVector getOutputVector();
		std::vector<_InputVector> createInputDataSet( uint num );
		std::vector<_OutputVector> createOutputDataSet( uint num );
	};

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::ArtificialNeuralNetwork( _Dimension layer_dims, _InternalDataType learning_rate )
	{
		learning_rate_ = (_InternalDataType) learning_rate;
		if ( layer_dims.size() >= 2 ) resize( layer_dims );
	}


	// virtual
	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::~ArtificialNeuralNetwork()
	{
		//
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	void ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::initializeRandomWeights( _InputDataType min, _InternalDataType max )
	{
		//printf( "initializeRandomWeights()\n" );
		if ( max < min )
		{
			fprintf( stderr, "For random weight initialization, the min value must be less than the max value\n" );
			return;
		}

		srand( time( NULL ) );
		for ( uint n = 0; n < layer_dims_.size() - 1; n++ )
		{
			for ( uint i = 0; i < nodes_[n].size(); i++ )
			{
				for ( uint j = 0; j < nodes_[n + 1].size(); j++ )
				{
					// don't try to change the weight for links going into bias nodes
					if ( nodes_[n + 1][j].type_ == Node<>::Type::bias ) continue;

					getWeight( n, i, j ) = min + ( max - min ) * (double) rand() / (double) RAND_MAX;
				}
			}
		}
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	void ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::resize( _Dimension & layer_dims )
	{
		// printf( "resize()\n" );
		if ( layer_dims.size() < 2 )
		{
			fprintf( stderr, "Invalid layer size; the network requires at least two layers.\n" );
			return;
		}

		layer_dims_ = layer_dims;

		nodes_.resize( layer_dims_.size() );
		weights_.resize( layer_dims_.size() - 1 );

		for ( uint n = 0; n < layer_dims_.size(); n++ )
		{
			nodes_[n].resize( layer_dims_[n] );


			// add bias nodes and update weight matrix dimensions
			if ( n < layer_dims_.size() - 1 )
			{
				// add input layer neurons
				if ( n == 0 )
				{
					for ( uint i = 0; i < nodes_[n].size(); i++ )
					{
						nodes_[n][i] = InputNode<_InternalDataType> ();
					}
				}

				// create the bias for all layers except the output layer
				nodes_[n].push_back( BiasNode<_InternalDataType> () );


				// the number of cols in the weight matrix is one less than the number of layers
				weights_[n].resize( nodes_[n].size() * layer_dims_[n + 1] );
			}


			// add output layer neurons
			if ( n == layer_dims_.size() - 1 )
			{
				nodes_[n].resize( layer_dims_[n] );

				for ( uint i = 0; i < nodes_[n].size(); i++ )
				{
					nodes_[n][i] = OutputNode<_InternalDataType> ();
				}
			}
		}
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	std::string ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::toString()
	{
		std::stringstream ss;
		for ( uint n = 0; n < nodes_.size(); n++ )
		{
			ss << "Nodes[" << n << "]: ";
			ss << "{ " << nodes_[n][0].type_ << "|" << nodes_[n][0].activation_state_;

			for ( size_t i = 1; i < nodes_[n].size(); i++ )
			{
				ss << ", " << nodes_[n][i].type_ << "|" << nodes_[n][i].activation_state_;
			}

			ss << " }\n";
		}

		for ( uint n = 0; n < weights_.size(); n++ )
		{
			ss << "Weights[" << n << "]: ";
			ss << weights_[n].toString() << "\n";
		}

		return ss.str();
	}


	// weight on link from node i in layer n to node j in layer n+1 is weights_[n][i * layer_dims_[n+1] + j]
	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	_InternalDataType & ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::getWeight( int n, int i, int j )
	{
		return weights_[n][i * layer_dims_[n + 1] + j];
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	Vector<_OutputDataType> ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::propagateData( _InputVector & input, bool scale_output )
	{
		//printf( "propagateData()\n" );

		//printf( "System state: \n%s", toString().c_str() );

		static _OutputVector result( layer_dims_[-1] );

		if ( input.size() != layer_dims_[0] )
		{
			fprintf( stderr, "Invalid data vector size; size must be %d\n", layer_dims_[0] );
			return result;
		}

		// load inputs
		for ( uint i = 0; i < layer_dims_[0]; i++ )
		{
			nodes_[0][i].stimulate( (double) input[i] );
		}

		// propagate data through the layers
		for ( uint n = 0; n < layer_dims_.size() - 1; n++ )
		{
			for ( uint j = 0; j < nodes_[n + 1].size(); j++ )
			{
				double total = 0;


				// if the node receiving the input is a bias, don't bother calculating its output does not depend on any input since it will be constant
				if ( nodes_[n + 1][j].type_ != Node<>::Type::bias )
				{
					for ( uint i = 0; i < nodes_[n].size(); i++ )
					{
						total += getWeight( n, i, j ) * nodes_[n][i].activation_state_;
					}
				}
				nodes_[n + 1][j].stimulate( total );
			}
		}

		//printf( "System state: \n%s", toString().c_str() );


		// dump outputs
		for ( uint i = 0; i < layer_dims_[-1]; i++ )
		{
			result[i] = (_OutputDataType) ( nodes_[nodes_.size() - 1][i].activation_state_ * ( scale_output ? output_scales_[i] : 1.0 ) );
		}

		return result;
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	Vector<_InternalDataType> ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::getErrorVector( _InputVector & input, _OutputVector & desired_output )
	{
		//printf( "getErrorVector()\n" );
		static _InternalDataVector result( layer_dims_[-1] );

		if ( input.size() != layer_dims_[0] || desired_output.size() != layer_dims_[-1] )
		{
			fprintf( stderr, "Invalid input/output vector size; input size must be %d; output size must be %d\n", layer_dims_[0], layer_dims_[-1] );
			return result;
		}

		_OutputVector output = propagateData( input, false );

		for ( uint i = 0; i < output.size(); i++ )
		{
			result[i] = (_InternalDataType) ( desired_output[i] / output_scales_[i] - output[i] );
		}

		return result;
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	_InternalDataType ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::getTotalSystemError( Vector<_InternalDataVector> & error )
	{
		//printf( "getTotalSystemError()\n" );

		_InternalDataType E = 0;

		for ( uint i = 0; i < error.size(); i++ )
		{
			if ( error[i].size() != layer_dims_[-1] )
			{
				fprintf( stderr, "Invalid error vector size %d; size must be %d\n", error[i].size(), layer_dims_[-1] );
				return 0;
			}

			for ( uint j = 0; j < error[i].size(); j++ )
			{
				E += pow( error[i][j], 2 );
			}
		}

		return E / ( 2.0 * (_InternalDataType) error.size() );
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	void ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::backpropagateErrorVector( _InternalDataVector & error )
	{
		//printf( "backpropagateErrorVector()\n" );

		if ( error.size() != layer_dims_[-1] )
		{
			fprintf( stderr, "Invalid error vector size; error size must be %d\n", layer_dims_[-1] );
			return;
		}

		// create one error gradient value per node
		static Vector<_InternalDataVector> error_gradient( layer_dims_.size() );
		for ( uint n = 0; n < error_gradient.size(); n++ )
		{
			error_gradient[n].resize( nodes_[n].size() );
		}

		//printf( ">> %d %d %d %f %f %d\n", error_gradient.size(), error_gradient[-1].size(), error.size(), error[0], error_gradient[-1][0], nodes_[-1].size() );

		//printf( "calculate error gradient for output layer neurons\n" );
		// calculate error gradient for output layer neurons
		for ( uint i = 0; i < error_gradient[-1].size(); i++ )
		{
			_InternalDataType & activation_state = nodes_[-1][i].activation_state_;
			error_gradient[-1][i] = activation_state * ( 1.0 - activation_state ) * error[i];
		}

		// calculate error gradient for neurons in all other layers
		for ( uint n = layer_dims_.size() - 1; n > 0; n-- )
		{
			// calculate error gradient for layer n-1
			for ( uint i = 0; i < nodes_[n - 1].size(); i++ )
			{
				//printf( "calculate error gradient for neuron %d in layer %d\n", i, n - 1 );
				_InternalDataType & activation_state = nodes_[n - 1][i].activation_state_;
				double total = 0;
				for ( uint j = 0; j < nodes_[n].size(); j++ )
				{
					// don't backpropagate the values of bias nodes
					if ( nodes_[n][j].type_ == Node<>::Type::bias ) continue;


					//printf( "total += w_%d_%d,%d * D_%d,%d\n", n - 1, i, j, n, j );
					//total += weights_[n - 1][i * nodes_[n].size() + j] * error_gradient[n][j];
					total += getWeight( n - 1, i, j ) * error_gradient[n][j];
				}
				error_gradient[n - 1][i] = activation_state * ( 1.0 - activation_state ) * total;
			}

		}

		for ( uint n = layer_dims_.size() - 1; n > 0; n-- )
		{
			//printf( "update weights between layer %d and layer %d\n", n - 1, n );
			// update weights between layer n-1 and layer n
			for ( uint i = 0; i < nodes_[n - 1].size(); i++ )
			{
				_InternalDataType & activation_state = nodes_[n - 1][i].activation_state_;
				for ( uint j = 0; j < nodes_[n].size(); j++ )
				{
					// don't try to change weights going into bias nodes
					if ( nodes_[n][j].type_ == Node<>::Type::bias ) continue;


					//printf( "update w_%d_%d,%d\n", n - 1, i, j );
					getWeight( n - 1, i, j ) += error_gradient[n][j] * activation_state * learning_rate_;
				}
			}
		}


		/*printf( "Error gradient for layer 1 %s\n", error_gradient[-1].toString().c_str() );
		 for ( uint n = layer_dims_.size() - 1; n > 0; n-- )
		 {
		 printf( "Error gradient for layer %d %s\n", n - 1, error_gradient[n - 1].toString().c_str() );
		 }
		 for ( uint n = layer_dims_.size() - 1; n > 0; n-- )
		 {
		 printf( "Weights for layer %d %s\n", n - 1, weights_[n - 1].toString().c_str() );
		 }*/
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	int ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::train( std::vector<_InputVector> & inputs, std::vector<_OutputVector> & outputs, double min_error,
			int max_iterations )
	{
		_InternalDataVector max_output_amplitude( layer_dims_[-1] );
		output_scales_.resize( layer_dims_[-1] );

		for ( uint i = 0; i < inputs.size(); i++ )
		{
			printf( "%s:", inputs[i].toString().c_str() );
			printf( "%s\n", outputs[i].toString().c_str() );

			for ( size_t j = 0; j < outputs[i].size(); j++ )
			{
				if ( fabs( outputs[i][j] ) > max_output_amplitude[j] ) max_output_amplitude[j] = fabs( outputs[i][j] );
			}
		}

		for ( size_t i = 0; i < max_output_amplitude.size(); i++ )
		{
			output_scales_[i] = max_output_amplitude[i] > 0.0 ? max_output_amplitude[i] : 1.0;
		}

		int iterations = 0;
		double total_error = 2 * min_error;

		Vector<_InternalDataVector> error_vector( inputs.size() );

		int progress_counter = 0;

		while ( total_error > min_error && iterations < max_iterations )
		{
			for ( uint i = 0; i < inputs.size(); i++ )
			{
				error_vector[i] = getErrorVector( inputs[i], outputs[i] );
				backpropagateErrorVector( error_vector[i] );
			}
			total_error = (double) getTotalSystemError( error_vector );
			//printf( "Total error: %f\n", total_error );
			iterations++;

			if( progress_counter > max_iterations / 20 )
			{
				progress_counter = 0;
				printf("Progress %f %f\n", 100.0 * (double) iterations / (double) max_iterations, total_error );
			}
			progress_counter ++;
		}

		int result = total_error <= min_error ? iterations : -1;

		printf( "\n\ntraining summary: " );

		if ( result >= 0 ) printf( "learned %d data pairs in %d iterations with an error of %f\n", inputs.size(), iterations, total_error );
		else printf( "failed to learn %d data pairs in %d iterations; error: %f\n", inputs.size(), iterations, total_error );

		for ( uint i = 0; i < inputs.size(); i++ )
		{
			printf( "%s : %s\n", inputs[i].toString().c_str(), propagateData( inputs[i] ).toString().c_str() );
		}

		return result;
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	Vector<_InputDataType> ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::getInputVector()
	{
		// printf( "Generating input vector\n" );
		_InputVector result;
		result.resize( layer_dims_[0] );
		return result;
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	Vector<_OutputDataType> ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::getOutputVector()
	{
		// printf( "Generating output vector\n" );
		_OutputVector result;
		result.resize( layer_dims_[-1] );
		return result;
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	std::vector<Vector<_InputDataType> > ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::createInputDataSet( uint num )
	{
		// printf( "Generating input dataset\n" );
		std::vector<_InputVector> result;
		result.resize( num );
		for ( uint i = 0; i < num; i++ )
		{
			result[i] = getInputVector();
		}

		return result;
	}

	template<class _InputDataType, class _OutputDataType, class _InternalDataType>
	std::vector<Vector<_OutputDataType> > ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>::createOutputDataSet( uint num )
	{
		// printf( "Generating output dataset\n" );
		std::vector<_OutputVector> result;
		result.resize( num );
		for ( uint i = 0; i < num; i++ )
		{
			result[i] = getOutputVector();
		}

		return result;
	}


	// workaround for 'templated typdef' problem
	template<typename _InputDataType = bool, typename _OutputDataType = bool, typename _InternalDataType = double>
	struct _ANN: ArtificialNeuralNetwork<_InputDataType, _OutputDataType, _InternalDataType>
	{

	};

}

#endif /* ARTIFICIAL_NEURAL_NETWORK_H_ */
