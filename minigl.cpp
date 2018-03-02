/**
 * minigl.cpp
 * -------------------------------
 * Implement miniGL here.
 *
 * You may include minigl.h and any of the standard C++ libraries.
 * No other includes are permitted.  Other preprocessing directives
 * are also not permitted.  These requirements are strictly
 * enforced.  Be sure to run a test grading to make sure your file
 * passes the sanity tests.
 *
 * The behavior of the routines your are implenting is documented here:
 * https://www.opengl.org/sdk/docs/man2/
 * Note that you will only be implementing a subset of this.  In particular,
 * you only need to implement enough to pass the tests in the suite.
 */

#include "minigl.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
#include <cstdio>
#include <stack>
#include <iostream>
#include <utility>

using namespace std;

const MGLfloat zeromatrix[16] = {0};
const MGLfloat identity[] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
const double PI = 3.141592653589793;


stack<vector<MGLfloat>> projviewstack;
stack<vector<MGLfloat>> modelviewstack;
vector<MGLfloat> currentmatrix(identity,identity+16);

MGLpoly_mode currentpolymode;
bool glbegin = false;
MGLmatrix_mode currentmatmode = MGL_MODELVIEW;
stack<vector<MGLfloat>>* currentstack = &modelviewstack; //current matrix stack is default to be modelview

vector<vector<MGLfloat>> vertices;
vector<vector<vector<MGLfloat>>> primitives;
vector<MGLfloat> currentcolor(3,0);

/**
 * Standard macro to report errors
 */
inline void MGL_ERROR(const char* description) {
    printf("%s\n", description);
    exit(1);
}

class Triangle
{
	private:
	
		vector<vector<MGLfloat>> vertices;
		MGLfloat area;
		MGLfloat minX,minY,maxX,maxY;
		
		MGLfloat AreaPixel(const vector<MGLfloat>& b, const vector<MGLfloat>& c, const MGLfloat& x, const MGLfloat& y)
		{
			return x*b[1]+b[0]*c[1]+c[0]*y-b[0]*y-c[0]*b[1]-x*c[1];
		}
		
		MGLpixel drawcolor(const MGLfloat& alpha, const MGLfloat& beta, const MGLfloat& gama)
		{
			MGLfloat orialpha, oribeta, origama;
			
			MGLfloat k = alpha/vertices[0][3] + beta/vertices[1][3] + gama/vertices[2][3];
			
			orialpha = alpha/vertices[0][3]/k;
			oribeta = beta/vertices[1][3]/k;
			origama = gama/vertices[2][3]/k;
			
			int red = (int) (0.5+255 * (orialpha*(vertices[0])[4]+oribeta*(vertices[1])[4]+origama*(vertices[2])[4]));
			int green = (int) (0.5+255 * (orialpha*(vertices[0])[5]+oribeta*(vertices[1])[5]+origama*(vertices[2])[5]));
			int blue = (int) (0.5+255 * (orialpha*(vertices[0])[6]+oribeta*(vertices[1])[6]+origama*(vertices[2])[6]));
			
			return Make_Pixel(red,green,blue);
		}
		
	public:
		
		Triangle(const vector<MGLfloat>& a, const vector<MGLfloat>& b, const vector<MGLfloat>& c)
		{
			vertices.push_back(a);
			minX = maxX = a[0];
			minY = maxY = a[1];
			
			vertices.push_back(b);
			minX = min(minX,b[0]);
			maxX = max(maxX,b[0]);
			minY = min(minY,b[1]);
			maxY = max(maxY,b[1]);
			
			vertices.push_back(c);
			minX = min(minX,c[0]);
			maxX = max(maxX,c[0]);
			minY = min(minY,c[1]);
			maxY = max(maxY,c[1]);
			
			area = a[0]*b[1]+b[0]*c[1]+c[0]*a[1]-b[0]*a[1]-c[0]*b[1]-a[0]*c[1];
		}
		
		vector<MGLfloat> getboundary()
		{
			vector<MGLfloat> result(4,0);
			result[0] = minX;
			result[1] = minY;
			result[2] = maxX;
			result[3] = maxY;
			
			return result;
		}
		
		MGLpixel drawpixel(const MGLfloat& x, const MGLfloat& y, MGLfloat& z)
		{
			MGLfloat alpha = AreaPixel(vertices[1], vertices[2], x, y)/area;
			MGLfloat beta = AreaPixel(vertices[2], vertices[0], x, y)/area;
			MGLfloat gama = AreaPixel(vertices[0], vertices[1], x, y)/area;		
			
			if(alpha>=0&&alpha<=1&&beta>=0&&beta<=1&&gama>=0&&gama<=1)
			{
				z = alpha*vertices[0][2]+beta*vertices[1][2] + gama*vertices[2][2];
				return drawcolor(alpha, beta, gama); //igore shared edges
			}
			else
			{
				return 0;
			}
		}
		
};


vector<MGLfloat> multibyvec(const vector<MGLfloat>& matrix, const vector<MGLfloat>& vec)
{
	vector<MGLfloat> result(4,0);
	for (unsigned int i = 0; i < 4; i += 1)
	{
		for (unsigned int j = 0; j < 4; j += 1)
		{
			result[i] += matrix[i+j*4] * vec[j];
		}
	}
	return result;
}

void transformation(const vector<MGLfloat>& mat, vector<MGLfloat>& v)
{
	if (mat.size()!=16)
	{
		MGL_ERROR("transformation matrix is wrong!");
	}

	vector<MGLfloat> vec(v.begin(),v.begin()+4);
	vec = multibyvec(mat,vec);
	
/*	
	for (unsigned int i = 0; i < 3; i += 1)
	{
		vec[i] = vec[i]/vec[3];
	}
*/  

	for (unsigned int i = 0; i < 4; i += 1)
	{
		v[i] = vec[i];
	}
}

MGLfloat dot(const int& clipplane, const vector<MGLfloat>& v)
{
	switch (clipplane)
	{
		case 0:
			return v[0] + v[3];        /* v * (1 0 0 1) */
		case 1:
			return -v[0] + v[3];      /* v * (-1 0 0 1) */
		case 2:
			return v[1] + v[3];        /* v * (0 1 0 1) */
		case 3:
			return -v[1] + v[3];      /* v * (0 -1 0 1) */
		case 4:
			return v[2] + v[3];        /* v * (0 0 1 1) */
		case 5:
			return -v[2] + v[3];      /* v * (0 0 -1 1) */
	}
	return 0;
}

uint outcode(const vector<MGLfloat>& v)
{
	uint outcode = 0;
	for (int i = 0; i < 6; ++i)
	{
		if (dot(i,v) < 0) outcode |= (1 << i);
	}
	return outcode;
}

vector<MGLfloat> clipline(int clipplane, const vector<MGLfloat>& v1, const vector<MGLfloat>& v2)
{
	MGLfloat dot1 = dot(clipplane, v1);
	MGLfloat dot2 = dot(clipplane, v2);
	vector<MGLfloat> new_vertex(7,0);

	MGLfloat s = -dot1/(dot2-dot1);
	
	if (s == 1 || s == 0)
	{
		new_vertex.clear();
		return new_vertex;
	}
	
	for (unsigned int i = 0; i < 7; i += 1)
	{
		new_vertex[i] = v1[i]+ s * (v2[i]-v1[i]); 
	}

	return new_vertex;

}

void divide_w(vector<MGLfloat>& v)
{
	for (unsigned int i = 0; i < 3; i += 1)
	{
		v[i] = v[i]/v[3];
	}
}

void clipping(vector<vector<MGLfloat>>& primitive, vector<Triangle>& triangles)
{
	
	stack<vector<vector<MGLfloat>>> primitive_stack;
	if (primitive.size()==3)
	{
	
		primitive_stack.push(primitive);
		
		while (!primitive_stack.empty())
		{
			vector<vector<MGLfloat>> tmp_primitive = primitive_stack.top();
			primitive_stack.pop();
			
			uint out0 = outcode(tmp_primitive[0]);
			uint out1 = outcode(tmp_primitive[1]);
			uint out2 = outcode(tmp_primitive[2]);
			
			if (!(out0|out1|out2))
			{
				divide_w(tmp_primitive[0]);
				divide_w(tmp_primitive[1]);
				divide_w(tmp_primitive[2]);
				Triangle triangle(tmp_primitive[0],tmp_primitive[1],tmp_primitive[2]);
				triangles.push_back(triangle);
				continue;	
			}
			
			if (out0&out1&out2)
			{
				continue;
			}
			
			vector<vector<MGLfloat>> tmp_vertices;
			vector<MGLfloat> tmp;	
			
			bool newpoint = false;
			
			for (unsigned int i = 0; i < 6; i += 1)
			{
			
				if (!(out0 & (1<<i)))
				{
					tmp_vertices.push_back(tmp_primitive[0]);
				}
				if ((out0 ^ out1) & (1<<i))
				{
					tmp = clipline(i, tmp_primitive[0], tmp_primitive[1]);
					if (!tmp.empty())
					{
						tmp_vertices.push_back(tmp);
						newpoint = true;
					}
				}
				if (!(out1 & (1<<i)))
				{
					tmp_vertices.push_back(tmp_primitive[1]);
				}
			
				if ((out1 ^ out2) & (1<<i))
				{
					tmp = clipline(i, tmp_primitive[1], tmp_primitive[2]);
					if (!tmp.empty())
					{
						tmp_vertices.push_back(tmp);
						newpoint = true;
					}
				}
			
				if (!(out2 & (1<<i)))
				{
					tmp_vertices.push_back(tmp_primitive[2]);
				}
			
				if ((out2 ^ out0) & (1<<i))
				{
					tmp = clipline(i, tmp_primitive[2], tmp_primitive[0]);
					if (!tmp.empty())
					{
						tmp_vertices.push_back(tmp);
						newpoint = true;
					}
				}
			
				vector<vector<MGLfloat>> tmp_pri;
				if (tmp_vertices.size() > 2 && newpoint == true)
				{
					for (unsigned int j = 2; j < tmp_vertices.size(); j += 1)
					{
						tmp_pri.push_back(tmp_vertices[0]);
						tmp_pri.push_back(tmp_vertices[j-1]);
						tmp_pri.push_back(tmp_vertices[j]);
						
						primitive_stack.push(tmp_pri);
					
						tmp_pri.clear();
					}
					break;
				}
				
				tmp_vertices.clear();	
			}
			
		}
	}
	
}

void clean()
{
	while(!projviewstack.empty()) projviewstack.pop();	
	while(!modelviewstack.empty()) modelviewstack.pop();
	currentmatmode = MGL_MODELVIEW;
	currentstack = &modelviewstack;
	primitives.clear();
	currentcolor = vector<MGLfloat>(3,0);
	currentmatrix = vector<MGLfloat>(identity,identity+16);
}


/**
 * Read pixel data starting with the pixel at coordinates
 * (0, 0), up to (width,  height), into the array
 * pointed to by data.  The boundaries are lower-inclusive,
 * that is, a call with width = height = 1 would just read
 * the pixel at (0, 0).
 *
 * Rasterization and z-buffering should be performed when
 * this function is called, so that the data array is filled
 * with the actual pixel values that should be displayed on
 * the two-dimensional screen.
 */
void mglReadPixels(MGLsize width,
                   MGLsize height,
                   MGLpixel *data)
{
	MGLfloat pixel_x = 2.0f/(width-1);
	MGLfloat pixel_y = 2.0f/(height-1);
	
	vector<Triangle> triangles;
	vector<MGLfloat> secondmat;
	
	if (currentmatmode == MGL_PROJECTION)
	{
		mglMatrixMode(MGL_MODELVIEW);		
		secondmat = currentmatrix;
		mglMatrixMode(MGL_PROJECTION);
	}
	else
	{
		mglMatrixMode(MGL_PROJECTION);
		secondmat = currentmatrix;
		mglMatrixMode(MGL_PROJECTION);
	}

	for (unsigned int i = 0; i < primitives.size(); i += 1)
	{
		for (unsigned int j = 0; j < primitives[i].size(); j += 1)
		{
			transformation(secondmat,primitives[i][j]);
		}
		if (primitives[i].size()==3)
		{
			clipping(primitives[i],triangles);
		}

	}
	
	uint minX,minY,maxX,maxY;
	vector<MGLfloat> boundary;
	vector<pair<MGLpixel, MGLfloat>> zbuffer(width*width, pair<MGLpixel, MGLfloat>(0xff,0));
	MGLpixel tmp = 0;
	MGLfloat z = 0;
	
	for (unsigned int k = 0; k < triangles.size(); k += 1)
	{
		boundary = triangles[k].getboundary();
		
		minX = (int) ((boundary[0]+1)/pixel_x);
		minY = (int) ((boundary[1]+1)/pixel_y);
		maxX = (int) ((boundary[2]+1)/pixel_x) + 1;
		maxY = (int) ((boundary[3]+1)/pixel_y) + 1;
		
		if(maxX > width) maxX = width;
		if(maxY > height) maxY = height;
		
		for (unsigned int j = minY; j < maxY; j += 1)
		{
			for (unsigned int i = minX; i < maxX; i += 1)
			{				
				if ( (tmp = triangles[k].drawpixel(i*pixel_x-1,j*pixel_y-1, z)) )
				{
					if (zbuffer[i+j*width].first == 0xff || zbuffer[i+j*width].second > z)
					{
						zbuffer[i+j*width] = pair<MGLpixel, MGLfloat>(tmp,z);
					}					
				}
			}
		}
	}
	
	for (unsigned int j = 0; j < height; j += 1)
	{
		for (unsigned int i = 0; i < width; i += 1)
		{
			data[i+j*width] = zbuffer[i+j*width].first;
		}
	}
	
	
	clean();
	
}

/**
 * Start specifying the vertices for a group of primitives,
 * whose type is specified by the given mode.
 */
void mglBegin(MGLpoly_mode mode)
{
	if (glbegin || !vertices.empty())
	{
		MGL_ERROR("mglEnd() is Missing");
	}
	glbegin = true;

	currentpolymode = mode;
}


/**
 * Stop specifying the vertices for a group of primitives.
 */
void mglEnd()
{
	vector<vector<MGLfloat>> primitive;
	if (!glbegin)
	{
		MGL_ERROR("mglBegin() is Missing");
	}
	
	glbegin = false;
	
	if (currentpolymode == MGL_TRIANGLES)
	{
		for (unsigned int i = 0; i < vertices.size(); i += 1)
		{
			transformation(currentmatrix, vertices[i]);
			primitive.push_back(vertices[i]);
			if (i % 3 == 2)
			{
				primitives.push_back(primitive);
				primitive.clear();
			}
		}
	}
	
	if (currentpolymode == MGL_QUADS)
	{
		for (unsigned int i = 0; i < vertices.size(); i += 1)
		{
			transformation(currentmatrix, vertices[i]);
			primitive.push_back(vertices[i]);
			if (i % 4 == 2 && (i+1) < vertices.size())
			{
				primitives.push_back(primitive);
				primitive[1] = primitive[2];
				transformation(currentmatrix, vertices[++i]);
				primitive[2] = vertices[i];				
				primitives.push_back(primitive);
				primitive.clear();
			}
		}
	}
	
	vertices.clear();
}

/**
 * Specify a two-dimensional vertex; the x- and y-coordinates
 * are explicitly specified, while the z-coordinate is assumed
 * to be zero.  Must appear between calls to mglBegin() and
 * mglEnd().
 */
void mglVertex2(MGLfloat x,
                MGLfloat y)
{
	if (!glbegin)
	{
		MGL_ERROR("Must appear between calls to mglBegin() and mglEnd().");
	}
	MGLfloat tmp[7] = {x,y,0,1,currentcolor[0],currentcolor[1],currentcolor[2]};
	vector<MGLfloat> vertex(tmp,tmp+7);
	vertices.push_back(vertex);
}

/**
 * Specify a three-dimensional vertex.  Must appear between
 * calls to mglBegin() and mglEnd().
 */
void mglVertex3(MGLfloat x,
                MGLfloat y,
                MGLfloat z)
{
	if (!glbegin)
	{
		MGL_ERROR("Must appear between calls to mglBegin() and mglEnd().");
	}
	
	
	MGLfloat tmp[7] = {x,y,z,1,currentcolor[0],currentcolor[1],currentcolor[2]};
	vector<MGLfloat> vertex(tmp,tmp+7);
	vertices.push_back(vertex);
}

/**
 * Set the current matrix mode (modelview or projection).
 */
void mglMatrixMode(MGLmatrix_mode mode)
{
	if(currentmatmode == mode) return;
	
	mglPushMatrix();
	
	if(mode == MGL_PROJECTION)
	{
		currentstack = &projviewstack;
	}
	else
	{
		currentstack = &modelviewstack;
	}
	
	mglPopMatrix();
		
	currentmatmode = mode;
}

/**
 * Push a copy of the current matrix onto the stack for the
 * current matrix mode.
 */
void mglPushMatrix()
{
	currentstack->push(currentmatrix);
}

/**
 * Pop the top matrix from the stack for the current matrix
 * mode.
 */
void mglPopMatrix()
{
	if(!currentstack->empty())
	{
		currentmatrix = currentstack->top();
		currentstack->pop();
	}
	else mglLoadIdentity(); 
}

/**
 * Replace the current matrix with the identity.
 */
void mglLoadIdentity()
{
	currentmatrix = vector<MGLfloat>(identity,identity+16);
}

/**
 * Replace the current matrix with an arbitrary 4x4 matrix,
 * specified in column-major order.  That is, the matrix
 * is stored as:
 *
 *   ( a0  a4  a8  a12 )
 *   ( a1  a5  a9  a13 )
 *   ( a2  a6  a10 a14 )
 *   ( a3  a7  a11 a15 )
 *
 * where ai is the i'th entry of the array.
 */
void mglLoadMatrix(const MGLfloat *matrix)
{
	currentmatrix = vector<MGLfloat>(matrix,matrix+16);
}

/**
 * Multiply the current matrix by an arbitrary 4x4 matrix,
 * specified in column-major order.  That is, the matrix
 * is stored as:
 *
 *   ( a0  a4  a8  a12 )
 *   ( a1  a5  a9  a13 )
 *   ( a2  a6  a10 a14 )
 *   ( a3  a7  a11 a15 )
 *
 * where ai is the i'th entry of the array.
 */
void mglMultMatrix(const MGLfloat *matrix)
{
	vector<MGLfloat> tmpmat(16,0);
	for(uint j=0; j<4; ++j)
	{
		for(uint i=0; i<4; ++i)
		{
			MGLfloat tmp = 0;
			for(uint k=0; k<4; ++k)
			{
				tmp += currentmatrix[i+k*4]*matrix[k+j*4]; //for AB=C, c_ij = Sum_k(a_ik * b_kj)
			}
			tmpmat[i+j*4] = tmp;
		}
	}
	
	currentmatrix = tmpmat;
}

/**
 * Multiply the current matrix by the translation matrix
 * for the translation vector given by (x, y, z).
 */
void mglTranslate(MGLfloat x,
                  MGLfloat y,
                  MGLfloat z)
{
	MGLfloat transmat[16]={0};
	transmat[0] = 1;
	transmat[5] = 1;
	transmat[10] = 1;
	transmat[12] = x;
	transmat[13] = y;
	transmat[14] = z;
	transmat[15] = 1;
	
	mglMultMatrix(transmat);	
}

/**
 * Multiply the current matrix by the rotation matrix
 * for a rotation of (angle) degrees about the vector
 * from the origin to the point (x, y, z).
 */
void mglRotate(MGLfloat angle,
               MGLfloat x,
               MGLfloat y,
               MGLfloat z)
{
	double l = sqrt(x*x+y*y+z*z);
	x = x/l;
	y = y/l;
	z = z/l;
	MGLfloat rotmat[16]={0};
	MGLfloat c = cos(angle*PI/180);
	MGLfloat s = sin(angle*PI/180);
	rotmat[0] = x*x*(1-c)+c;
	rotmat[1] = y*x*(1-c)+z*s;
	rotmat[2] = x*z*(1-c)-y*s;
	rotmat[4] = x*y*(1-c)-z*s;
	rotmat[5] = y*y*(1-c)+c;
	rotmat[6] = y*z*(1-c)+x*s;
	rotmat[8] = x*z*(1-c)+y*s;
	rotmat[9] = y*z*(1-c)-x*s;
	rotmat[10] = z*z*(1-c)+c;
	rotmat[15] = 1;
	
	mglMultMatrix(rotmat);
}

/**
 * Multiply the current matrix by the scale matrix
 * for the given scale factors.
 */
void mglScale(MGLfloat x,
              MGLfloat y,
              MGLfloat z)
{
	MGLfloat scalemat[16]={0};
	scalemat[0] = x;
	scalemat[5] = y;
	scalemat[10] = z;
	scalemat[15] = 1;
	
	mglMultMatrix(scalemat);
}

/**
 * Multiply the current matrix by the perspective matrix
 * with the given clipping plane coordinates.
 */
void mglFrustum(MGLfloat left,
                MGLfloat right,
                MGLfloat bottom,
                MGLfloat top,
                MGLfloat near,
                MGLfloat far)
{
	MGLfloat perspmat[16]={0};
	perspmat[0] = 2*near/(right-left);
	perspmat[5] = 2*near/(top-bottom);
	perspmat[8] = (right+left)/(right-left);
	perspmat[9] = (top+bottom)/(top-bottom);
	perspmat[10] = (near+far)/(near-far);
	perspmat[11] = -1;
	perspmat[14] = 2*near*far/(near-far); 
	
	mglMultMatrix(perspmat);
}

/**
 * Multiply the current matrix by the orthographic matrix
 * with the given clipping plane coordinates.
 */
void mglOrtho(MGLfloat left,
              MGLfloat right,
              MGLfloat bottom,
              MGLfloat top,
              MGLfloat near,
              MGLfloat far)
{
	MGLfloat orthomat[16]={0};
	orthomat[0] = 2/(right-left);
	orthomat[5] = 2/(top-bottom);
	orthomat[10] = 2/(near-far); 
	orthomat[12] = -(right+left)/(right-left);
	orthomat[13] = -(top+bottom)/(top-bottom);
	orthomat[14] = -(near+far)/(near-far);
	orthomat[15] = 1;
	
	mglMultMatrix(orthomat);
}

/**
 * Set the current color for drawn shapes.
 */
void mglColor(MGLfloat red,
              MGLfloat green,
              MGLfloat blue)
{
	if (red < 0) red = 0;
	if (green < 0) green = 0;
	if (blue < 0) blue = 0;
	currentcolor[0] = red;
	currentcolor[1] = green;
	currentcolor[2] = blue;	
}
