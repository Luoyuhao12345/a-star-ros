#ifndef NODE2D_H
#define NODE2D_H

class Node2D
{
	public:
		Node2D(): Node2D(0, 0, 0, 0, 0, 0, nullptr) {}
		Node2D(int x_index, int y_index, float x, float y, float g, float h, Node2D* pred)
		{
			this->x_index = x_index;
			this->y_index = y_index;
			this->x = x;
			this->y = y;	
			this->g = g;
			this->h = h;
			this->pred = pred;
		}
	public:
		int x_index;
		int y_index;
		float x;
		float y;
		float g;
		float h;
		float f;
		Node2D* pred;
};

#endif