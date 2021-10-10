#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
	int num_height_points, float thickness) {
	this->width = width;
	this->height = height;
	this->num_width_points = num_width_points;
	this->num_height_points = num_height_points;
	this->thickness = thickness;

	buildGrid();
	buildClothMesh();
}

Cloth::~Cloth() {
	point_masses.clear();
	springs.clear();

	if (clothMesh) {
		delete clothMesh;
	}
}

void Cloth::buildGrid() {
	// (Part 1): Build a grid of masses and springs.
	Vector3D start = Vector3D(0, 0, 0);
	double i_step = height / num_height_points;
	double j_step = width / num_width_points;
	srand(time(NULL));

	for (uint16_t i = 0; i < num_height_points; i++)
	{
		for (uint16_t j = 0; j < num_width_points; j++)
		{
			double z = 1.0 / ((rand() % 10000) + 1000);
			Vector3D position = start + Vector3D(
				i * i_step,
				(!orientation) ? 1 : j * j_step,
				(!orientation) ? j * j_step : z);
			bool is_pinned = false;
			for (auto ID : pinned)
				is_pinned = is_pinned || (ID[0] == i && ID[1] == j);
			point_masses.emplace_back(position, is_pinned);
		}
	}
	PointMass* iter = &point_masses[0];
	for (uint16_t i = 0; i < num_height_points; i++)
	{
		for (uint16_t j = 0; j < num_width_points; j++)
		{
			if (j > 0)
				springs.emplace_back(iter, iter - 1, STRUCTURAL);
			if (i > 0)
				springs.emplace_back(iter, iter - num_width_points, STRUCTURAL);
			if (i > 0 && j > 0)
				springs.emplace_back(iter, iter - num_width_points - 1, SHEARING);
			if (i > 0 && j < num_width_points - 1)
				springs.emplace_back(iter, iter - num_width_points + 1, SHEARING);
			if (j > 1)
				springs.emplace_back(iter, iter - 2, BENDING);
			if (i > 1)
				springs.emplace_back(iter, iter - 2 * num_width_points, BENDING);
			iter++;
		}
	}
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters* cp,
	vector<Vector3D> external_accelerations,
	vector<CollisionObject*>* collision_objects) {
	double mass = width * height * cp->density / num_width_points / num_height_points;
	double delta_t = 1.0f / frames_per_sec / simulation_steps;

	// (Part2): Compute total force acting on each point mass.
	for (auto s = springs.begin(); s < springs.end(); s++)
	{
		if ((s->spring_type == STRUCTURAL && !cp->enable_structural_constraints) ||
			(s->spring_type == SHEARING && !cp->enable_shearing_constraints) ||
			(s->spring_type == BENDING && !cp->enable_bending_constraints)) {
			continue;
		}

		Vector3D v = s->pm_a->position - s->pm_b->position;
		double dist = v.norm();
		bool extend = dist > s->rest_length;
		double ks_force = cp->ks * (dist - s->rest_length);
		if (!extend)
			ks_force *= 1.0;
		if (s->spring_type == BENDING)
			ks_force *= 0.2;
		s->pm_a->forces -= ks_force * v / dist;
		s->pm_b->forces += ks_force * v / dist;
	}

	// (Part 2): Use Verlet integration to compute new point mass positions
	build_spatial_map();
	for (auto node = point_masses.begin(); node < point_masses.end(); node++)
	{
		if (node->pinned) continue;
		for (int i = 0; i < external_accelerations.size(); i++)
		{
			node->forces += mass * external_accelerations[i];
		}
		Vector3D temp = node->position;
		node->position = node->position + (1 - cp->damping / 100) * (node->position - node->last_position) + node->forces / mass * delta_t * delta_t;
		node->last_position = temp;
		node->forces *= 0;//clear forces

		// (Part 4): Handle self-collisions.
		self_collide(*node, simulation_steps);

		// (Part 3): Handle collisions with other primitives.
		for (int i = 0; i < collision_objects->size(); i++)
		{
			auto ptr = *(collision_objects + i)->begin();
			ptr->collide(*node);
		}
	}
	// (Part 2): Constrain the changes to be such that the spring does not change
	// in length more than 10% per timestep [Provot 1995].
	for (auto s = springs.begin(); s < springs.end(); s++)
	{
		if ((s->spring_type == STRUCTURAL && !cp->enable_structural_constraints) ||
			(s->spring_type == SHEARING && !cp->enable_shearing_constraints) ||
			(s->spring_type == BENDING && !cp->enable_bending_constraints)) {
			continue;
		}

		Vector3D v = s->pm_a->position - s->pm_b->position;
		double dist = v.norm();
		double max_nu = 0.1;
		double nu = (dist - s->rest_length) / s->rest_length;
		if (nu > max_nu)
		{
			double fix_dist = dist - (max_nu + 1.0) * s->rest_length;
			if (!(s->pm_a->pinned && s->pm_b->pinned))
			{
				if (s->pm_a->pinned)
				{
					s->pm_b->position += fix_dist * v;
				}
				else if (s->pm_b->pinned)
				{
					s->pm_a->position -= fix_dist * v;
				}
				else
				{
					s->pm_a->position -= fix_dist * v / 2;
					s->pm_b->position += fix_dist * v / 2;
				}
			}
		}
	}
}

void Cloth::build_spatial_map() {
	for (const auto& entry : map) {
		delete(entry.second);
	}
	map.clear();

	// (Part 4): Build a spatial map out of all of the point masses.
	for (int p = 0; p < point_masses.size(); p++) {
		PointMass* ptr = &point_masses[p];
		float hkey = hash_position(ptr->position);

		if (!map.count(hkey)) { //if it is already not in map using unordered map's count  
			map[hkey] = new vector<PointMass*>(); //for a new key dynamically allocate memory for a point mass* vector
		}
		map[hkey]->push_back(ptr);
	}
}

void Cloth::self_collide(PointMass& pm, double simulation_steps) {
	// (Part 4): Handle self-collision for a given point mass.
	float hkey = hash_position(pm.position);
	if (map.count(hkey)) { //if key exists in hashtable
		Vector3D avg_correction(0, 0, 0);//apply corrections
		int num_corrections = 0;
		vector<PointMass*> hashedPMs = *map[hkey];
		for (int p = 0; p < hashedPMs.size(); p++) {
			PointMass* candidatePM = hashedPMs[p]; //get every point mass in current map[hash]'s pm vector
			if (candidatePM->position == pm.position) //to prevent self collision
				continue;
			Vector3D PtoC = candidatePM->position - pm.position;
			double separation = PtoC.norm(); //magnitude of vector from PM pos to candidate PM pos
			double exceededDist = 2.0 * thickness - separation;
			if (exceededDist > 0) { //too close, add contribution for this candidate PM to correction vector
				Vector3D CtoPdir = pm.position - candidatePM->position;
				CtoPdir.normalize(); //just need direction
				avg_correction += (exceededDist * CtoPdir);
				num_corrections++;
			}
		}
		avg_correction /= (double)num_corrections; //to get average
		if (num_corrections > 0) { //there are some corrections to be made (particles self colliding)
			pm.position += avg_correction / simulation_steps; //apply avg correction scaled down by simulation steps
		}
	}
}

float Cloth::hash_position(Vector3D pos) {
	// (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
	float h = 3 * height / num_height_points; //3 empirically proven constant for spatial hashing accuracy
	float w = 3 * width / num_width_points;
	float t = (h > w) ? h : w;

	float x_id = (pos.x - fmod(pos.x, w)) / w; //truncate x coordinate with remainder of pos.x/w to get to closest 3Dbox coords and divide over w (xrange dimension of box)
	float y_id = (pos.y - fmod(pos.y, h)) / h;
	float z_id = (pos.z - fmod(pos.z, t)) / t;

	float result = x_id + y_id * h + z_id * w * h; // for an order sensitive, unique hash 
	return result;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
	PointMass* pm = &point_masses[0];
	for (int i = 0; i < point_masses.size(); i++) {
		pm->position = pm->start_position;
		pm->last_position = pm->start_position;
		pm++;
	}
}

void Cloth::buildClothMesh() {
	if (point_masses.size() == 0) return;

	ClothMesh* clothMesh = new ClothMesh();
	vector<Triangle*> triangles;

	// Create vector of triangles
	for (int y = 0; y < num_height_points - 1; y++) {
		for (int x = 0; x < num_width_points - 1; x++) {
			PointMass* pm = &point_masses[y * num_width_points + x];
			// Get neighboring point masses:
			/*                      *
			 * pm_A -------- pm_B   *
			 *             /        *
			 *  |         /   |     *
			 *  |        /    |     *
			 *  |       /     |     *
			 *  |      /      |     *
			 *  |     /       |     *
			 *  |    /        |     *
			 *      /               *
			 * pm_C -------- pm_D   *
			 *                      *
			 */

			float u_min = x;
			u_min /= num_width_points - 1;
			float u_max = x + 1;
			u_max /= num_width_points - 1;
			float v_min = y;
			v_min /= num_height_points - 1;
			float v_max = y + 1;
			v_max /= num_height_points - 1;

			PointMass* pm_A = pm;
			PointMass* pm_B = pm + 1;
			PointMass* pm_C = pm + num_width_points;
			PointMass* pm_D = pm + num_width_points + 1;

			Vector3D uv_A = Vector3D(u_min, v_min, 0);
			Vector3D uv_B = Vector3D(u_max, v_min, 0);
			Vector3D uv_C = Vector3D(u_min, v_max, 0);
			Vector3D uv_D = Vector3D(u_max, v_max, 0);


			// Both triangles defined by vertices in counter-clockwise orientation
			triangles.push_back(new Triangle(pm_A, pm_C, pm_B,
				uv_A, uv_C, uv_B));
			triangles.push_back(new Triangle(pm_B, pm_C, pm_D,
				uv_B, uv_C, uv_D));
		}
	}

	// For each triangle in row-order, create 3 edges and 3 internal halfedges
	for (int i = 0; i < triangles.size(); i++) {
		Triangle* t = triangles[i];

		// Allocate new halfedges on heap
		Halfedge* h1 = new Halfedge();
		Halfedge* h2 = new Halfedge();
		Halfedge* h3 = new Halfedge();

		// Allocate new edges on heap
		Edge* e1 = new Edge();
		Edge* e2 = new Edge();
		Edge* e3 = new Edge();

		// Assign a halfedge pointer to the triangle
		t->halfedge = h1;

		// Assign halfedge pointers to point masses
		t->pm1->halfedge = h1;
		t->pm2->halfedge = h2;
		t->pm3->halfedge = h3;

		// Update all halfedge pointers
		h1->edge = e1;
		h1->next = h2;
		h1->pm = t->pm1;
		h1->triangle = t;

		h2->edge = e2;
		h2->next = h3;
		h2->pm = t->pm2;
		h2->triangle = t;

		h3->edge = e3;
		h3->next = h1;
		h3->pm = t->pm3;
		h3->triangle = t;
	}

	// Go back through the cloth mesh and link triangles together using halfedge
	// twin pointers

	// Convenient variables for math
	int num_height_tris = (num_height_points - 1) * 2;
	int num_width_tris = (num_width_points - 1) * 2;

	bool topLeft = true;
	for (int i = 0; i < triangles.size(); i++) {
		Triangle* t = triangles[i];

		if (topLeft) {
			// Get left triangle, if it exists
			if (i % num_width_tris != 0) { // Not a left-most triangle
				Triangle* temp = triangles[i - 1];
				t->pm1->halfedge->twin = temp->pm3->halfedge;
			}
			else {
				t->pm1->halfedge->twin = nullptr;
			}

			// Get triangle above, if it exists
			if (i >= num_width_tris) { // Not a top-most triangle
				Triangle* temp = triangles[i - num_width_tris + 1];
				t->pm3->halfedge->twin = temp->pm2->halfedge;
			}
			else {
				t->pm3->halfedge->twin = nullptr;
			}

			// Get triangle to bottom right; guaranteed to exist
			Triangle* temp = triangles[i + 1];
			t->pm2->halfedge->twin = temp->pm1->halfedge;
		}
		else {
			// Get right triangle, if it exists
			if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
				Triangle* temp = triangles[i + 1];
				t->pm3->halfedge->twin = temp->pm1->halfedge;
			}
			else {
				t->pm3->halfedge->twin = nullptr;
			}

			// Get triangle below, if it exists
			if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
				Triangle* temp = triangles[i + num_width_tris - 1];
				t->pm2->halfedge->twin = temp->pm3->halfedge;
			}
			else {
				t->pm2->halfedge->twin = nullptr;
			}

			// Get triangle to top left; guaranteed to exist
			Triangle* temp = triangles[i - 1];
			t->pm1->halfedge->twin = temp->pm2->halfedge;
		}

		topLeft = !topLeft;
	}

	clothMesh->triangles = triangles;
	this->clothMesh = clothMesh;
}
