/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		//std::copy(point.begin(),point.end(),std::ostream_iterator<float>(std::cout, " "));

		if (root==NULL)
		{
			 root = new Node(point, id);
		}
		else
		{
			int layer=0;
		 	bool assigned=0;
			Node* Node_1 = new Node(point,id);
			Node* Node_2 = root;
			while(assigned==0)
			{
			if(Node_1->point[layer%3]<Node_2->point[layer%3])
			{
				if(Node_2->left==NULL)
				{
				Node_2->left=Node_1;
				assigned=1;
				}
				else
				{
					Node_2=Node_2->left;
					layer++;
					continue;
				}
			}
			else
			{
				if(Node_2->right==NULL)
                {
				Node_2->right=Node_1;
				assigned=1;
				}
				else
				{
					Node_2=Node_2->right;
					layer++;
					continue;
				}
			}
			}
		
		}
	}



	// return a list of point ids in the tree that are within distance of target
	float calculateDistance(std::vector<float> point1, std::vector<float> point2)
	{
		return sqrt(pow((point1[0]-point2[0]),2)+pow((point1[1]-point2[1]),2)+ pow((point1[2]-point2[2]),2));
	}

	void searchHelper(std::vector<float> target, Node* Node, float distanceTol, int layer, std::vector<int> &ids)
	{
		if (Node!=NULL)
		{
		if(abs(Node->point[0]-target[0])<=distanceTol && abs(Node->point[1]-target[1])<=distanceTol && abs(Node->point[2]-target[2])<=distanceTol) 
		{
			if(calculateDistance(Node->point,target)<distanceTol)
					ids.push_back(Node->id);
		}
			if ((target[layer%3]-distanceTol)<Node->point[layer%3])
				searchHelper(target, Node->left, distanceTol, layer+1, ids);  
			if (( target[layer%3]+distanceTol)>Node->point[layer%3])
				searchHelper(target, Node->right, distanceTol, layer+1, ids); 
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, distanceTol, 0, ids);
		return ids;
	}

};




