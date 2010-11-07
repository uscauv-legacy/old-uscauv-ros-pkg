/*******************************************************************************
 *
 *      test_ann
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

#include <libvml/vml.h>

int main( int argc, char ** argv )
{
	typedef double _InputDataType;
	typedef double _OutputDataType;

	int iterations = 0;

	vml::_Dimension dim;

	dim.resize( 3 );
	dim[0] = 2;
	dim[1] = 3;
	dim[2] = 1;

	vml::ArtificialNeuralNetwork<_InputDataType, _OutputDataType> ann( dim, 0.75 );
	printf( "initial structure:\n%s", ann.toString().c_str() );
	ann.initializeRandomWeights( -1.0, 1.0 );

	std::vector<vml::_ANN<_InputDataType, _OutputDataType>::_InputVector> inputs = ann.createInputDataSet( 4 );
	std::vector<vml::_ANN<_InputDataType, _OutputDataType>::_OutputVector> outputs = ann.createOutputDataSet( 4 );

	inputs[0][0] = 0.0;
	inputs[0][1] = 0.0;
	outputs[0][0] = 0.0;

	inputs[1][0] = 0.0;
	inputs[1][1] = 1.0;
	outputs[1][0] = 1.0;

	inputs[2][0] = 1.0;
	inputs[2][1] = 0.0;
	outputs[2][0] = 1.0;

	inputs[3][0] = 1.0;
	inputs[3][1] = 1.0;
	outputs[3][0] = 0.0;

	Vector<vml::_ANN<_InputDataType, _OutputDataType>::_InternalDataVector> error_vector;
	error_vector.resize( inputs.size() );

	double total_error = 1.0;

	while ( total_error > 0.01 )
	{
		for ( uint i = 0; i < inputs.size(); i++ )
		{
			error_vector[i] = ann.getErrorVector( inputs[i], outputs[i] );

			//printf( "error_vector: %s\n", error_vector[i].toString().c_str() );

			ann.backpropagateErrorVector( error_vector[i] );

			//printf( "%s : %s\n", inputs[i].toString().c_str(), ann.propagateData( inputs[i] ).toString().c_str() );
		}

		total_error = (double) ann.getTotalSystemError( error_vector );
		//printf( "\n--------------------\n" );
		printf( "Total error: %f\n", total_error );

		iterations++;
	}

	printf( "\nfinished in %d iterations\n", iterations );
	printf( "--------------------\n" );

	printf( "final structure:\n%s", ann.toString().c_str() );

	for ( uint i = 0; i < inputs.size(); i++ )
	{
		printf( "%s : %s\n", inputs[i].toString().c_str(), ann.propagateData( inputs[i] ).toString().c_str() );
	}


	/*output_vector = ann.propagateData( input_vector );
	 printf( "%s\n", output_vector.toString().c_str() );

	 error_vector = ann.getErrorVector( input_vector, desired_output_vector );*/

}
