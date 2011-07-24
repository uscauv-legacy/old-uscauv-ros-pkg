#include "CVSuperPixel.H"

typedef struct
{
	int rank;
	int p;
	int size;
} uni_elt;

class universe
{
public:
	universe( int elements );
	~universe();
	int find( int x );
	void join( int x, int y );
	int size( int x ) const
	{
		return elts[x].size;
	}
	int num_sets() const
	{
		return num;
	}

private:
	uni_elt *elts;
	int num;
};

universe::universe( int elements )
{
	elts = new uni_elt[elements];
	num = elements;
	for ( int i = 0; i < elements; i++ )
	{
		elts[i].rank = 0;
		elts[i].size = 1;
		elts[i].p = i;
	}
}

universe::~universe()
{
	delete[] elts;
}

int universe::find( int x )
{
	int y = x;
	while ( y != elts[y].p )
		y = elts[y].p;
	elts[x].p = y;
	return y;
}

void universe::join( int x, int y )
{
	if ( elts[x].rank > elts[y].rank )
	{
		elts[y].p = x;
		elts[x].size += elts[y].size;
	}
	else
	{
		elts[x].p = y;
		elts[y].size += elts[x].size;
		if ( elts[x].rank == elts[y].rank ) elts[y].rank++;
	}
	num--;
}

// threshold function
#define THRESHOLD(size, c) (c/size)

typedef struct
{
	float w;
	int a, b;
} edge;

bool operator<( const edge &a, const edge &b )
{
	return a.w < b.w;
}

/*
 * Segment a graph
 *
 * Returns a disjoint-set forest representing the segmentation.
 *
 * num_vertices: number of vertices in graph.
 * num_edges: number of edges in graph
 * edges: array of edges.
 * c: constant for treshold function.
 */
universe *segment_graph( int num_vertices, int num_edges, edge *edges, float c )
{
	// sort edges by weight
	std::sort( edges, edges + num_edges );

	// make a disjoint-set forest
	universe *u = new universe( num_vertices );

	// init thresholds
	float *threshold = new float[num_vertices];
	for ( int i = 0; i < num_vertices; i++ )
		threshold[i] = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	for ( int i = 0; i < num_edges; i++ )
	{
		edge *pedge = &edges[i];

		// components connected by this edge
		int a = u->find( pedge->a );
		int b = u->find( pedge->b );
		if ( a != b )
		{
			if ( ( pedge->w <= threshold[a] ) && ( pedge->w <= threshold[b] ) )
			{
				u->join( a, b );
				a = u->find( a );
				threshold[a] = pedge->w + THRESHOLD(u->size(a), c);
			}
		}
	}

	// free up
	delete threshold;
	return u;
}

// dissimilarity measure between pixels
float diff( cv::Mat im, int x1, int y1, int x2, int y2 )
{
	return sqrt(
			pow( im.at<cv::Vec3b> ( y1, x1 )[0] - im.at<cv::Vec3b> ( y2, x2 )[0], 2.0 ) + pow( im.at<cv::Vec3b> ( y1, x1 )[1] - im.at<cv::Vec3b> ( y2, x2 )[1], 2.0 ) + pow(
					im.at<cv::Vec3b> ( y1, x1 )[2] - im.at<cv::Vec3b> ( y2, x2 )[2], 2.0 ) );
}

std::vector<std::vector<cv::Point> > SuperPixelSegment( cv::Mat im, int sigma, float c, int min_size )
{
	if(sigma > 0)
	  cv::blur( im, im, cv::Size( sigma, sigma ) );

	int width = im.size().width;
	int height = im.size().height;

	// build graph
	edge *edges = new edge[width * height * 4];
	int num = 0;
	for ( int y = 0; y < height; y++ )
	{
		for ( int x = 0; x < width; x++ )
		{
			if ( x < width - 1 )
			{
				edges[num].a = y * width + x;
				edges[num].b = y * width + ( x + 1 );
				edges[num].w = diff( im, x, y, x + 1, y );
				num++;
			}

			if ( y < height - 1 )
			{
				edges[num].a = y * width + x;
				edges[num].b = ( y + 1 ) * width + x;
				edges[num].w = diff( im, x, y, x, y + 1 );
				num++;
			}

			if ( ( x < width - 1 ) && ( y < height - 1 ) )
			{
				edges[num].a = y * width + x;
				edges[num].b = ( y + 1 ) * width + ( x + 1 );
				edges[num].w = diff( im, x, y, x + 1, y + 1 );
				num++;
			}

			if ( ( x < width - 1 ) && ( y > 0 ) )
			{
				edges[num].a = y * width + x;
				edges[num].b = ( y - 1 ) * width + ( x + 1 );
				edges[num].w = diff( im, x, y, x + 1, y - 1 );
				num++;
			}
		}
	}

	// segment
	universe *u = segment_graph( width * height, num, edges, c );

	// post process small components
	for ( int i = 0; i < num; i++ )
	{
		int a = u->find( edges[i].a );
		int b = u->find( edges[i].b );
		if ( ( a != b ) && ( ( u->size( a ) < min_size ) || ( u->size( b ) < min_size ) ) ) u->join( a, b );
	}
	delete[] edges;
	int num_ccs = u->num_sets();

	std::vector<std::vector<cv::Point2i> > groups( u->num_sets() );

	// The group numbers created by the algorithm are more or less random. We
	// need to compress them so we can easily insert the group points into
	// vectors.
	std::map<int, int> groupLabels;
	int labelCount = 0;

	for ( int y = 0; y < height; y++ )
		for ( int x = 0; x < width; x++ )
		{
			int groupUnorderedId = u->find( y * width + x );

			// If we haven't seen this group before, create a new ordered label for it
			if ( groupLabels.find( groupUnorderedId ) == groupLabels.end() ) groupLabels[groupUnorderedId] = labelCount++;

			int groupOrderedId = groupLabels[groupUnorderedId];

			groups[groupOrderedId].push_back( cv::Point( x, y ) );
		}

	delete u;

	return groups;
}

// ######################################################################
cv::Mat SuperPixelDebugImage( std::vector<std::vector<cv::Point> > const& groups, cv::Mat originalImage )
{
	cv::Mat debugImage( originalImage.size(), originalImage.type() );

	for ( size_t grpIdx = 0; grpIdx < groups.size(); grpIdx++ )
	{
		cv::Vec3f avgColor( 0, 0, 0 );
		for ( size_t pntIdx = 0; pntIdx < groups[grpIdx].size(); pntIdx++ )
			avgColor += originalImage.at<cv::Vec3b> ( groups[grpIdx][pntIdx] );
		avgColor[0] /= groups[grpIdx].size();
		avgColor[1] /= groups[grpIdx].size();
		avgColor[2] /= groups[grpIdx].size();
		for ( size_t pntIdx = 0; pntIdx < groups[grpIdx].size(); pntIdx++ )
			debugImage.at<cv::Vec3b> ( groups[grpIdx][pntIdx] ) = avgColor;
	}

	return debugImage;
}

