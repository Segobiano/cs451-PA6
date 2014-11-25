#include "raytracer.h"
#include <algorithm>
#include <cfloat>
#include "GL/gliLight.h"
#define Epsilon 0.00001
//some helper functions
inline double clamp(double x){ return x<0 ? 0 : x>1 ? 1 : x; }

inline int toInt(double x){ return int( clamp(x)*255 + .5); }


//constructor
RayTracer::RayTracer(list<model>& models) : m_models(models)
{

	modelmax = new GLdouble[16];
	project = new GLdouble[16];
	view = new GLint[4];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelmax);
	glGetDoublev(GL_PROJECTION_MATRIX, project);
	glGetIntegerv(GL_VIEWPORT, view);
	scx = new GLdouble(0);
	scy = new GLdouble(0);
	scz = new GLdouble(0);
	scxf = new GLdouble(0);
	scyf = new GLdouble(0);
	sczf = new GLdouble(0);
	camx = gli::getCameraPos()[0];
	camy = gli::getCameraPos()[1];
	camz = gli::getCameraPos()[2];
	
}

inline void show_progress_bar(double ratio)
{
	// Show the load bar.
	static const int w = 50;
	int   c = ratio * w;

	cout << setw(3) << (int)(ratio * 100) << "% [";
	for (int x = 0; x<c; x++) cout << "=";
	for (int x = c; x<w; x++) cout << " ";
	cout << "]\r" << flush;
}

//render an image with "nray_per_pixel" rays created per pixel
void RayTracer::render(unsigned int img_w, unsigned int img_h, unsigned int nray_per_pixel)
{
	mImage_h = img_h;
	mImage_w = img_w;
	

	//create a black image of size img_w X img_h
	m_img = vector< vector< Vector3d > >(img_h, vector< Vector3d >(img_w, Vector3d(0,0,0) ));

	//generate rays
	for (int y = 0; y < img_h; y++)
	{

		for (int x = 0; x < img_w; x++)
		{
			Vector3d color;
			for (int n = 0; n < nray_per_pixel; n++)
			{
				
				Ray r=create_a_random_ray(x, y);

				pair<model *, triangle *> X = intersect(r);

				//determine the color of this ray 
				if (X.first != NULL && X.second != NULL)
				{
					Vector3d rc = raycolor(*X.first, X.second, r);
					color = rc + color;
				}
			}

			m_img[y][x] = color / nray_per_pixel;
		}//end of x

		show_progress_bar(y*1.0 / img_h);

	}//end of y

	cout << "\nRendering is done." << endl;
}

// render an image with "nray_per_pixel" rays created per pixel
// create this ray randomly in the given pixel (x,y)
Ray RayTracer::create_a_random_ray(unsigned int x, unsigned int y)
{
	
	Ray r;
	
	drand48();
	double xvalue = x / mImage_w*(double)view[2] + (double)drand48();
	double yvalue = (view[3] - y) / mImage_h*(double)view[3] + (double)drand48();

	gluUnProject(xvalue, yvalue, 0, modelmax, project, view, scx, scy, scz);
	gluUnProject(xvalue, yvalue, 1, modelmax, project, view, scxf, scyf, sczf);
	
	
	Vector3d pPoint=Vector3d(*scx,*scy, *scz);
	Vector3d pPointf = Vector3d(*scxf, *scyf, *sczf);
	
	Vector3d camOrg = Vector3d(camx, camy, camz);

	//vector between the camera and the plane
	r.v = ((pPointf - pPoint).normalize());

	//so we start on the front clipping plane
	r.o = pPoint+camOrg;
	//all_rays.push_back(r);
	
	return r;
	
}

//returns a model and triangle that intersect ray r
//return pair<NULL,NULL> if no intersection is found
pair<model *, triangle *> RayTracer::intersect(Ray r)
{
	double min_dist = FLT_MAX;
	triangle * closest_T = NULL;
	model * closest_M = NULL;

	for (list<model>::iterator i = m_models.begin(); i != m_models.end(); i++)
	{
		triangle * t = closest_intersect(*i, r);
		if (t != NULL)
		{
			Point3d x;
			intersect(*i, t, r, x);
			double dist = (x - r.o).normsqr();
			if (dist < min_dist)
			{
				inter = x.get();
				min_dist = dist;
				closest_T = t;
				closest_M = &*i;
			}
		}
	}
	return make_pair(closest_M, closest_T);
}

//returns a triangle in model m that intersect ray r
//return NULL if no intersection is found
triangle * RayTracer::intersect(model& m, Ray r)
{
	for (int i = 0; i < m.t_size; i++)
	{
		Point3d x;
		if (intersect(m, &m.tris[i], r, x))
			inter = x.get();
			return &m.tris[i];
	}

	return NULL;
}

//returns a triangle in model m that make closest intersection with ray r
//return NULL if no intersection is found
triangle * RayTracer::closest_intersect(model& m, Ray r)
{
	double min_dist = FLT_MAX;
	triangle * closest_T=NULL;
	for (int i = 0; i < m.t_size; i++)
	{
		Point3d x;
		if (intersect(m, &m.tris[i], r, x))
		{
			double dist = (x - r.o).normsqr();
			if (dist < min_dist)
			{
				//inter = x.get();
				closest_T = &m.tris[i];
				min_dist = dist;
			}
		}//end if
	}//end for i

	return closest_T;
}

// determine if there is an intersection between triangle t and ray r
// return true if there is an intersection and store the intersection in x
// return false otherwise and x is undefined in this case
bool RayTracer::intersect(model& m, triangle * t, Ray r, Point3d& x)
{
	
	Vector3d v0 = m.vertices[t->v[0]].p.get();
	Vector3d v1 = m.vertices[t->v[1]].p.get();
	Vector3d v2 = m.vertices[t->v[2]].p.get();

	Vector3d mEdg1, mEdg2, mHvec, mSvec, mQvec;
	float mAnum, mFnum, mUnum, mVnum, parat;
	//constructing edges between the vertices
	 mEdg1 = v1 - v0;
	 mEdg2 = v2 - v0;

	mHvec = r.v%mEdg2;
	mAnum = mEdg1*mHvec;

	//if we are in range of epsilon since 0 could not be hit
	if ((mAnum > -.000001) && (mAnum < .000001)){
		//no intersection
		return false;
	}
	//inverse
	mFnum = 1 / mAnum;

	//both are points so they the - operater is defined for them
	mSvec = r.o - v0;

	//scaled by mFnum
	mUnum = mFnum*(mSvec*mHvec);

	//outside of 0 and 1 then no intersecction
	if (mUnum<0.0 || mUnum>1.0){
		return false;
	}
	//cross product between the edge and the origin and a point on the triangle
	mQvec = mSvec%mEdg1;
	mVnum = mFnum*(r.v*mQvec);

	if (mVnum<0.0 || (mUnum + mVnum)>1.0){
		//it is missing the intersection on the other side
		return false;
	}

	//compute the variable of the parametric
	double paraT = mFnum*(mEdg2*mQvec);

	//intersection happened
	if (paraT > .000001){
		//find the point on of intersection and return true and put the intersection in x
		
		x = ((r.o + r.v*paraT));
		return true;
	}
	
	return false;
}

//
// determine the color of ray r, by analizing the intersection between t and r 
// 
Vector3d RayTracer::raycolor(model& m, triangle * t, Ray r)
{
	
	// The area of a triangle is 
	Vector3d color = m.mat_color;
	Vector3d lightPos = Vector3d(light0_position[0], light0_position[1], light0_position[2]);
	vertex v0 = m.vertices[t->v[0]];
	vertex v1 = m.vertices[t->v[1]];
	vertex v2 = m.vertices[t->v[2]];
	Vector3d bary;
	//TODO: implement this
	//Interpolate the normal
	float areaABC = t->n * ((Vector3d(v1.p.get()) - Vector3d(v0.p.get())) % (Vector3d(v2.p.get()) - Vector3d(v0.p.get())));
	float areaPBC = t->n * ((Vector3d(v1.p.get()) - inter) % (Vector3d(v2.p.get()) - inter));
	float areaPCA = t->n * ((Vector3d(v2.p.get()) - inter) % (Vector3d(v0.p.get()) - inter));

	bary[0] = areaPBC / areaABC; 
	bary[1] = areaPCA / areaABC; 
	bary[2] = 1.0f - bary[0] - bary[1]; 
	//Vector3d weights = getBarycentricCoordinatesAt(Vector3d(pos.get()), Vector3d(v0.p.get()), Vector3d(v1.p.get()), Vector3d(v2.p.get()), t->n);
	Vector3d interNorm = (bary[0] * v0.n) + (bary[1] * v1.n) + (bary[2] * v2.n);
	//Get the diffusion value
	Vector3d fragToLight = (lightPos - inter).normalize();
	float lightval = fragToLight * interNorm;

	//Get the specular value
	Vector3d camOrg = Vector3d(camx, camy, camz);
	Vector3d eyeVec = ((lightPos - inter).normalize() + (camOrg-inter).normalize()).normalize();
	Vector3d specLight = m.mat_specular;
	if (eyeVec*interNorm > 0){
		specLight =specLight* pow((eyeVec*interNorm), m.mat_shininess);
	}
	else{
		specLight = specLight*0;
	}

	//is the frag facing away from the light?
	if (lightval<0){

		return color * 0;
	}
	//the intersection is in shadow
	if (inshadow()){
		return color / 2;
	}
	else{
		return color*lightval+specLight;
	}
	
}

//check if a point p is in shadow
bool RayTracer::inshadow()
{
	Ray lightRay;
	Vector3d lightPos = Vector3d(light0_position[0], light0_position[1], light0_position[2]);
	lightRay.v = (lightPos - inter).normalize();
	lightRay.o = inter + lightRay.v*.0001;
	Point3d x;
	bool found = false;

	//find intersectoin
	pair<model *, triangle *> X = intersect(lightRay);

	//check if that is closer then the light
	if (X.first != NULL && X.second != NULL){
		if ((lightRay.o - inter).norm() < (lightRay.o - lightPos).norm()){
			return true;
		}
	}

	return false;

}

//save rendered image to file
bool RayTracer::save2file(const string& filename)
{
	FILE *f = fopen(filename.c_str(), "w");         // Write image to PPM file.

	int h = m_img.size();
	if (h == 0) return true; //nothing to save...

	int w = m_img.front().size();
	
	fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);

	for (int i = 0; i < h; i++) 
		for (int j = 0; j < w; j++) 
			fprintf(f, "%d %d %d ", toInt(m_img[i][j][0]), toInt(m_img[i][j][1]), toInt(m_img[i][j][2]));

	fclose(f);
}




