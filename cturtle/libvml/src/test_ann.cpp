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
#include <stdio.h>
#include <string>

typedef double _InputDataType;
typedef double _OutputDataType;
typedef vml::ArtificialNeuralNetwork<_InputDataType, _OutputDataType, double> _ANN;

struct Point
{
	double x_;
	double y_;
	Point( double x, double y )
	{
		x_ = x;
		y_ = y;
	}
};

void dumpOutputs( FILE * out, _ANN & ann, Point i1_domain, Point i2_domain, double step )
{
	fprintf( out, "----------\nintput1,input2,output\n" );
	for ( double i = i1_domain.x_; i <= i1_domain.y_; i += step )
	{
		for ( double j = i2_domain.x_; j <= i2_domain.y_; j += step )
		{
			static _ANN::_InputVector input = ann.getInputVector();
			input[0] = i;
			input[1] = j;
			fprintf( out, "%f,%f,%f\n", i, j, ann.propagateData( input )[0] );
		}
	}
}

int main( int argc, char ** argv )
{
	FILE * out = fopen( "../docs/data.txt", "w+" );

	vml::_Dimension dim;

	dim.resize( 4 );
	dim[0] = 3;
	dim[1] = 6;
	dim[2] = 6;
	dim[3] = 1;

	_ANN ann( dim, 0.75 );
	printf( "initial structure:\n%s", ann.toString().c_str() );
	ann.initializeRandomWeights( -1.0, 1.0 );


	//dumpOutputs( out, ann );

	std::vector<_ANN::_InputVector> inputs = ann.createInputDataSet( 4 );
	std::vector<_ANN::_OutputVector> outputs = ann.createOutputDataSet( 4 );

	inputs[0][0] = 56.0;
	inputs[0][1] = 156.0;
	inputs[0][2] = 18.0;
	outputs[0][0] = 1.0;

	inputs[1][0] = 0.0;
	inputs[1][1] = 254.0;
	inputs[1][2] = 0.0;
	outputs[1][0] = 1.0;

	inputs[2][0] = 82.0;
	inputs[2][1] = 118.0;
	inputs[2][2] = 147.0;
	outputs[2][0] = 3.0;

	inputs[3][0] = 90.0;
	inputs[3][1] = 178.0;
	inputs[3][2] = 252.0;
	outputs[3][0] = 3.0;

	/*inputs[4][0] = 39.0;
	inputs[4][1] = 0.0;
	inputs[4][2] = 0.0;
	outputs[4][0] = 0.0;

	inputs[5][0] = 34.0;
	inputs[5][1] = 0.0;
	inputs[5][2] = 95.0;
	outputs[5][0] = 1.0;

	inputs[6][0] = 103.0;
	inputs[6][1] = 193.0;
	inputs[6][2] = 0.0;
	outputs[6][0] = 1.0;

	inputs[7][0] = 1.0;
	inputs[7][1] = 1.0;
	inputs[7][2] = 1.0;
	outputs[7][0] = 1.0;*/

	int iterations = ann.train( inputs, outputs, 0.01 );
	if ( iterations > 0 ) printf( "\nfinished in %d iterations\n", iterations );
	else printf( "\ntraining failed\n" );

	printf( "--------------------\n" );

	printf( "final structure:\n%s", ann.toString().c_str() );

	for ( uint i = 0; i < inputs.size(); i++ )
	{
		printf( "%s : %s\n", inputs[i].toString().c_str(), ann.propagateData( inputs[i] ).toString().c_str() );
	}

	dumpOutputs( out, ann, Point( -8.0, 8.0 ), Point( -8.0, 8.0 ), 0.025 );
	//dumpOutputs( out, ann, Point(-10.0, 10.0), Point(-10.0, 10.0), 0.1 );


	/*output_vector = ann.propagateData( input_vector );
	 printf( "%s\n", output_vector.toString().c_str() );

	 error_vector = ann.getErrorVector( input_vector, desired_output_vector );*/

	fclose( out );

	vml::AnnParser::saveAnnToFile( ann, "/home/edward/workspace/seabee3-ros-pkg/libvml/docs/ann_dump.txt" );

	_ANN loaded_ann;

	vml::AnnParser::readAnnFromFile( loaded_ann, "/home/edward/workspace/seabee3-ros-pkg/libvml/docs/ann_dump.txt" );

	vml::AnnParser::saveAnnToFile( loaded_ann, "/home/edward/workspace/seabee3-ros-pkg/libvml/docs/ann_dump2.txt" );


	//_ANN loaded_ann = vml::AnnParser::readAnnFromFile( std::string("/home/edward/workspace/seabee3-ros-pkg/libvml/docs/ann_dump.txt").c_str() );

	return 0;
}
