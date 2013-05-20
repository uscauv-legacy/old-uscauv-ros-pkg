#include "search_node.h"
#include "hough_line_transform.h"

using namespace std;

bool matchNodes(SearchNode, vector<SearchNode>);
void addSuccessorsToOpenList(vector<SearchNode>, vector<SearchNode> &, vector<SearchNode> &);
void expandNode(SearchNode, vector<Intersect>, vector<SearchNode> &);
SearchNode nextNode(vector<SearchNode> &, double);
void drawDetectedRectangles(vector<SearchNode>, cv::Mat);

int main(int argc, char** argv)
{
	cv::Mat image_src_, image_dst_;
	double theta_, theta_error_;
	
	if(argc < 4){ 
		fprintf(stderr, "ERROR: Specify input filename, search theta, and theta error.");
		return 0;
	}
	
	// Read in args
	image_src_ = cv::imread(argv[1], 0);	
	theta_ = atof(argv[2]);
	theta_error_ = atof(argv[3]);
	
	if(image_src_.empty())
	{
		fprintf(stderr, "ERROR: Cannot open image file.");
		return 0;
	}
	
	HoughLineTransform transform_(image_src_);
	transform_.applyHoughLineTransform();
	imshow("Hough Lines", transform_.getImageDstColor());
	cv::waitKey(0);
	
	if(transform_.getIntersectsSize() > 4) 
	{
	
		// Data
		vector<SearchNode> successor_nodes_;
		vector<SearchNode> open_nodes_;
		vector<SearchNode> closed_nodes_;
		vector<SearchNode> rectangles_;
	
		vector<Intersect> initial_intersect_;
		
		SearchNode initial_node_(transform_.getIntersects(0), initial_intersect_);

		SearchNode search_node_ = initial_node_;
		open_nodes_.push_back(search_node_);

		// Find a solution using manhattan distance
		// While there are unexpanded nodes:
		while(!open_nodes_.empty()){
			printf("Number of nodes in open_nodes: %d \n", open_nodes_.size());	
		
			// Expand (find the successors of) the current node
			expandNode(search_node_, transform_.getIntersects(), successor_nodes_);
			printf("Node expanded.\n");
			// Add the successors to the open list
			addSuccessorsToOpenList(successor_nodes_, open_nodes_, closed_nodes_);
			printf("Successors added to open list.\n");
			// Move the current node (the one just expanded) to the closed list
			closed_nodes_.push_back(search_node_);
			printf("Current node closed.\n");
			// Pick a new node with theta nearest 90 degrees to expand next
			search_node_ = nextNode(open_nodes_, theta_);
			printf("New node found.\n");
			// If the new node is the goal state, store it
			if(search_node_.isRectangle(theta_error_, theta_))
			{
				rectangles_.push_back(search_node_);
			}
		}
		
		printf("Number of rectangles detected: %d \n", rectangles_.size());
	
		image_dst_ = transform_.getImageSrc();
		drawDetectedRectangles(rectangles_, image_dst_);
	
		//imshow("Source", transform.image_src);
		//imshow("Hough Lines", transform.image_dst_color);
		//imshow("Rectangles", image_dst_);
		
		//cv::waitKey(0);
	
		return 0;
	}
	else 
	{
		printf("Not enough intersections detected. \n");
	}
}

// Compare successor node to all closed or open nodes
bool matchNodes(SearchNode successor, vector<SearchNode> nodes)
{
	for(vector<SearchNode>::iterator it = nodes.begin(); it != nodes.end(); ++it)
	{
		if(successor == *it)
		{		
			return true;
		}
	}
	return false;
}

// Adds list of successor nodes to open nodes
void addSuccessorsToOpenList(vector<SearchNode> successor, vector<SearchNode> &open, vector<SearchNode> &closed)
{
	for(int i = 0; i < successor.size(); i++)
	{		
		if((!matchNodes(successor[i], closed)) &&	
			(!matchNodes(successor[i], open)))
		{
			printf("Pushing back successor: ");
			successor[i].getIntersect().print("");
			open.push_back(successor[i]);
		}
	}
}

//TODO: Investigate this function
// Expand a node
void expandNode(SearchNode search, vector<Intersect> intersects, vector<SearchNode> &successor)
{
	successor.clear();
	vector<Intersect> valid_intersects = search.findValidIntersects(intersects);
	printf("Number of valid intersect successors: %d \n", valid_intersects.size());
	for(int i = 0; i < valid_intersects.size(); i++)
	{
		search.addToCorners(search.getIntersect());
		SearchNode node(valid_intersects[i], search.getCorners());
		successor.push_back(node);
	}
}

// Determines the next node to expand using value of theta
SearchNode nextNode(vector<SearchNode> &open, double theta)
{
	int index = 0;
	for(int i = 0; i < open.size(); i++){
		if((open[i].differenceFromAngle(theta) + open[i].getCornersSize())<
		(open[index].differenceFromAngle(theta)) + open[index].getCornersSize()){
			index = i;
		}
	}
	SearchNode return_node = open[index];
	open.erase(open.begin() + index);
	return return_node;
}	

// Draw detected rectangles on image
void drawDetectedRectangles(vector<SearchNode> rectangles, cv::Mat image)
{
	for(int i = 0; i < rectangles.size(); i++)
	{
		line(image, rectangles[i].getCorners(0).getIntersect(), 
			 rectangles[i].getCorners(1).getIntersect(), 
			 cv::Scalar(0, 255, 0), 3, CV_AA);
		line(image, rectangles[i].getCorners(1).getIntersect(), 
			 rectangles[i].getCorners(2).getIntersect(), 
			 cv::Scalar(0, 255, 0), 3, CV_AA);
		line(image, rectangles[i].getCorners(2).getIntersect(), 
			 rectangles[i].getCorners(3).getIntersect(), 
			 cv::Scalar(0, 255, 0), 3, CV_AA);
		line(image, rectangles[i].getCorners(3).getIntersect(), 
			 rectangles[i].getCorners(0).getIntersect(), 
			 cv::Scalar(0, 255, 0), 3, CV_AA);
		circle(image, rectangles[i].getCorners(0).getIntersect(), 
			   3, cv::Scalar(255,0,0), -1, 8, 0);
		circle(image, rectangles[i].getCorners(1).getIntersect(), 
			   3, cv::Scalar(255,0,0), -1, 8, 0);
		circle(image, rectangles[i].getCorners(2).getIntersect(), 
			   3, cv::Scalar(255,0,0), -1, 8, 0);
		circle(image, rectangles[i].getCorners(3).getIntersect(), 
			   3, cv::Scalar(255,0,0), -1, 8, 0);
	}
}
