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
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(nullptr)
	{}

	~KdTree()
	{
		delete root;
	}

	/** helper function inserts node into tree */
	void insertNode(Node** parent, Node* newNode, uint depth) {

        if (*parent == nullptr) {
            *parent = newNode;
            return;
        }

        // based on depth compare x or y coordinate
        uint coordIdx = depth % 2;

        // left - right based on y-value
        if (newNode->point.at(coordIdx) >= (*parent)->point.at(coordIdx)) {
            insertNode(&((*parent)->right), newNode, depth + 1);
        }
        else {
            insertNode(&((*parent)->left), newNode, depth + 1);
        }
	}

	/**
	 * Insert a new point into the tree
	 *
	 * @param point
	 * @param id
	 */
	void insert(std::vector<float> point, int id)
	{
		// create a new node
		Node* newNode = new Node(std::move(point), id);

		// place the newly created node correctly with in the root
		insertNode(&root, newNode, 0);
	}

    /** helper function searches tree for points in the neighborhood of a target point. */
    void searchTree(Node* parent, std::vector<int>& ids, std::vector<float> target, float distanceTol, uint depth) {

        if (parent == nullptr) {
            return;
        }

        // based on depth compare x or y coordinate
        uint coordIdx = depth % 2;

        // compare target's x-range or y-range
        float lowerBound = target[coordIdx] - distanceTol;
        float upperBound = target[coordIdx] + distanceTol;

        // test if parent is in box
        if(parent->point[coordIdx] < lowerBound) {
          searchTree(parent->right, ids, target, distanceTol, depth + 1);
        }
        else if (parent->point[coordIdx] > upperBound) {
          searchTree(parent->left, ids, target, distanceTol, depth + 1);
        }
        else {
           // check the other dimension (coordIdx + 1 mod 2) to complete the check that parent is in the 2*distanceTol
           // square neighborhood of target.
           uint otherIdx = (coordIdx + 1) % 2;
           bool inBox = parent->point[otherIdx] > (target[otherIdx] - distanceTol) && parent->point[otherIdx] < (target[otherIdx] + distanceTol);

           if (inBox) {
             float x_minus_tx = parent->point[0] - target[0];
             float y_minus_ty = parent->point[1] - target[1];

             // parent might be in the distanceTol-Disc around target point, so test it.
             float dist = x_minus_tx * x_minus_tx + y_minus_ty * y_minus_ty;
             if (dist < distanceTol * distanceTol)
               ids.push_back(parent->id);
           }

           //Otherwise, continue traversing the tree.
           searchTree(parent->left, ids, target, distanceTol, depth + 1);
           searchTree(parent->right, ids, target, distanceTol, depth + 1);
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchTree(root, ids, std::move(target), distanceTol, 0);

		return ids;
	}
	

};




