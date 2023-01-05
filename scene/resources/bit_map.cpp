/**************************************************************************/
/*  bit_map.cpp                                                           */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "bit_map.h"

#include "core/io/image_loader.h"
#include "core/templates/local_vector.h"
#include "core/templates/rb_set.h"
#include "core/variant/typed_array.h"

#include "core/string/string_builder.h"

void BitMap::create(const Size2i &p_size) {
	ERR_FAIL_COND(p_size.width < 1);
	ERR_FAIL_COND(p_size.height < 1);

	ERR_FAIL_COND(static_cast<int64_t>(p_size.width) * static_cast<int64_t>(p_size.height) > INT32_MAX);

	Error err = bitmask.resize(Math::division_round_up(p_size.width * p_size.height, 8));
	ERR_FAIL_COND(err != OK);

	width = p_size.width;
	height = p_size.height;

	memset(bitmask.ptrw(), 0, bitmask.size());
}

void BitMap::create_from_image_alpha(const Ref<Image> &p_image, float p_threshold) {
	ERR_FAIL_COND(p_image.is_null() || p_image->is_empty());
	Ref<Image> img = p_image->duplicate();
	img->convert(Image::FORMAT_LA8);
	ERR_FAIL_COND(img->get_format() != Image::FORMAT_LA8);

	create(Size2i(img->get_width(), img->get_height()));

	const uint8_t *r = img->get_data().ptr();
	uint8_t *w = bitmask.ptrw();

	for (int i = 0; i < width * height; i++) {
		int bbyte = i / 8;
		int bbit = i % 8;
		if (r[i * 2 + 1] / 255.0 > p_threshold) {
			w[bbyte] |= (1 << bbit);
		}
	}
}

void BitMap::set_bit_rect(const Rect2i &p_rect, bool p_value) {
	Rect2i current = Rect2i(0, 0, width, height).intersection(p_rect);
	uint8_t *data = bitmask.ptrw();

	for (int i = current.position.x; i < current.position.x + current.size.x; i++) {
		for (int j = current.position.y; j < current.position.y + current.size.y; j++) {
			int ofs = width * j + i;
			int bbyte = ofs / 8;
			int bbit = ofs % 8;

			uint8_t b = data[bbyte];

			if (p_value) {
				b |= (1 << bbit);
			} else {
				b &= ~(1 << bbit);
			}

			data[bbyte] = b;
		}
	}
}

int BitMap::get_true_bit_count() const {
	int ds = bitmask.size();
	const uint8_t *d = bitmask.ptr();
	int c = 0;

	// Fast, almost branchless version.

	for (int i = 0; i < ds; i++) {
		c += (d[i] & (1 << 7)) >> 7;
		c += (d[i] & (1 << 6)) >> 6;
		c += (d[i] & (1 << 5)) >> 5;
		c += (d[i] & (1 << 4)) >> 4;
		c += (d[i] & (1 << 3)) >> 3;
		c += (d[i] & (1 << 2)) >> 2;
		c += (d[i] & (1 << 1)) >> 1;
		c += d[i] & 1;
	}

	return c;
}

void BitMap::set_bitv(const Point2i &p_pos, bool p_value) {
	set_bit(p_pos.x, p_pos.y, p_value);
}

void BitMap::set_bit(int p_x, int p_y, bool p_value) {
	ERR_FAIL_INDEX(p_x, width);
	ERR_FAIL_INDEX(p_y, height);

	int ofs = width * p_y + p_x;
	int bbyte = ofs / 8;
	int bbit = ofs % 8;

	uint8_t b = bitmask[bbyte];

	if (p_value) {
		b |= (1 << bbit);
	} else {
		b &= ~(1 << bbit);
	}

	bitmask.write[bbyte] = b;
}

bool BitMap::get_bitv(const Point2i &p_pos) const {
	return get_bit(p_pos.x, p_pos.y);
}

bool BitMap::get_bit(int p_x, int p_y) const {
	ERR_FAIL_INDEX_V(p_x, width, false);
	ERR_FAIL_INDEX_V(p_y, height, false);

	int ofs = width * p_y + p_x;
	int bbyte = ofs / 8;
	int bbit = ofs % 8;

	return (bitmask[bbyte] & (1 << bbit)) != 0;
}

Size2i BitMap::get_size() const {
	return Size2i(width, height);
}

void BitMap::_set_data(const Dictionary &p_d) {
	ERR_FAIL_COND(!p_d.has("size"));
	ERR_FAIL_COND(!p_d.has("data"));

	create(p_d["size"]);
	bitmask = p_d["data"];
}

Dictionary BitMap::_get_data() const {
	Dictionary d;
	d["size"] = get_size();
	d["data"] = bitmask;
	return d;
}

Vector<Vector<Vector2>> BitMap::_march_square(const Rect2i &p_rect, const Point2i &p_start) const {
	int stepx = 0;
	int stepy = 0;
	int prevx = 0;
	int prevy = 1;
	int startx = p_start.x;
	int starty = p_start.y;
	int curx = startx;
	int cury = starty + 1;
	unsigned int count = 0;

	HashMap<Point2i, int> cross_map;

	// Add first two points to vector, as initial movement is always down.
	Vector<Vector2> _points{ Vector2(startx, starty), Vector2(curx, cury) };
	int points_size = 2;

	Vector<Vector<Vector2>> ret;

	// Add starting entry at start of return.
	ret.resize(1);

	while (true) {
		int sv = 0;
		{ // Square value

			/*
			checking the 2x2 pixel grid, assigning these values to each pixel, if not transparent
			+---+---+
			| 1 | 2 |
			+---+---+
			| 4 | 8 | <- current pixel (curx,cury)
			+---+---+
			*/
			Point2i tl = Point2i(curx - 1, cury - 1);
			sv += (p_rect.has_point(tl) && get_bitv(tl)) ? 1 : 0;
			Point2i tr = Point2i(curx, cury - 1);
			sv += (p_rect.has_point(tr) && get_bitv(tr)) ? 2 : 0;
			Point2i bl = Point2i(curx - 1, cury);
			sv += (p_rect.has_point(bl) && get_bitv(bl)) ? 4 : 0;
			Point2i br = Point2i(curx, cury);
			sv += (p_rect.has_point(br) && get_bitv(br)) ? 8 : 0;
			ERR_FAIL_COND_V(sv == 0 || sv == 15, Vector<Vector<Vector2>>());
		}

		switch (sv) {
			case 1:
			case 5:
			case 13:
				/* going UP with these cases:
				1          5           13
				+---+---+  +---+---+  +---+---+
				| 1 |   |  | 1 |   |  | 1 |   |
				+---+---+  +---+---+  +---+---+
				|   |   |  | 4 |   |  | 4 | 8 |
				+---+---+  +---+---+  +---+---+
				*/
				stepx = 0;
				stepy = -1;
				break;

			case 8:
			case 10:
			case 11:
				/* going DOWN with these cases:
				8          10         11
				+---+---+  +---+---+  +---+---+
				|   |   |  |   | 2 |  | 1 | 2 |
				+---+---+  +---+---+  +---+---+
				|   | 8 |  |   | 8 |  |   | 8 |
				+---+---+  +---+---+  +---+---+
				*/
				stepx = 0;
				stepy = 1;
				break;

			case 4:
			case 12:
			case 14:
				/* going LEFT with these cases:
				4          12         14
				+---+---+  +---+---+  +---+---+
				|   |   |  |   |   |  |   | 2 |
				+---+---+  +---+---+  +---+---+
				| 4 |   |  | 4 | 8 |  | 4 | 8 |
				+---+---+  +---+---+  +---+---+
				*/
				stepx = -1;
				stepy = 0;
				break;

			case 2:
			case 3:
			case 7:
				/* going RIGHT with these cases:
				2          3          7
				+---+---+  +---+---+  +---+---+
				|   | 2 |  | 1 | 2 |  | 1 | 2 |
				+---+---+  +---+---+  +---+---+
				|   |   |  |   |   |  | 4 |   |
				+---+---+  +---+---+  +---+---+
				*/
				stepx = 1;
				stepy = 0;
				break;
			case 9:
				/* Going DOWN if coming from the LEFT, otherwise go UP.
				9
				+---+---+
				| 1 |   |
				+---+---+
				|   | 8 |
				+---+---+
				*/

				if (prevx == 1) {
					stepx = 0;
					stepy = 1;
				} else {
					stepx = 0;
					stepy = -1;
				}
				break;
			case 6:
				/* Going RIGHT if coming from BELOW, otherwise go LEFT.
				6
				+---+---+
				|   | 2 |
				+---+---+
				| 4 |   |
				+---+---+
				*/

				if (prevy == -1) {
					stepx = 1;
					stepy = 0;
				} else {
					stepx = -1;
					stepy = 0;
				}
				break;
			default:
				ERR_PRINT("this shouldn't happen.");
		}

		// Handle crossing points.
		if (sv == 6 || sv == 9) {
			const Point2i cur_pos(curx, cury);

			// Find if this point has occurred before.
			if (HashMap<Point2i, int>::Iterator found = cross_map.find(cur_pos)) {
				// Find the point with the smallest y and smallest x (in that order!) to ensure proper form.
				int min_i = found->value + 1;

				for (int i = found->value + 2; i < points_size; ++i) {
					if (_points[i].y < _points[min_i].y || (_points[i].y == _points[min_i].y && _points[i].x < _points[min_i].x)) {
						min_i = i;
					}
				}

				Vector<Vector2> new_points;
				new_points.resize(points_size - (found->value + 1));
				Vector2 *ptrw = new_points.ptrw();

				// Add points from minimum to end.
				for (int i = min_i; i < points_size; ++i) {
					*ptrw++ = _points[i];
				}

				// Add points from start to minimum.
				for (int i = found->value + 1; i < min_i; ++i) {
					*ptrw++ = _points[i];
				}

				ret.push_back(new_points);

				// Remove points after crossing point.
				points_size = found->value + 1;

				// Erase trailing map elements.
				while (cross_map.last() != found) {
					cross_map.remove(cross_map.last());
				}

				cross_map.erase(cur_pos);
			} else {
				// Add crossing point to map.
				cross_map.insert(cur_pos, points_size - 1);
			}
		}

		curx += stepx;
		cury += stepy;

		if (curx == startx && cury == starty) {
			// If was going the same direction, drop last point.
			if (stepx == prevx && stepy == prevy) {
				points_size--;
			}

			break;
		}

		// Small optimization:
		// If the previous direction is same as the current direction,
		// then we should modify the last vector to current.
		if (stepx == prevx && stepy == prevy) {
			_points.set(points_size - 1, Vector2(curx, cury) - p_rect.position);
		} else {
			_points.resize(MAX(points_size + 1, _points.size()));
			_points.set(points_size, Vector2(curx, cury) - p_rect.position);
			points_size++;
		}

		count++;
		prevx = stepx;
		prevy = stepy;

		ERR_FAIL_COND_V((int)count > 2 * (width * height + 1), Vector<Vector<Vector2>>());
	} while (curx != startx || cury != starty);

	// Add remaining points to result.
	_points.resize(points_size);

	ret.set(0, _points);

	return ret;
}

struct PointEntry {
	Vector2 dir;
	int ind;
};

template <bool Flip>
struct PointEntryCmp {
	static int quadrant(const Vector2 &p_dir) {
		if (p_dir.y <= 0 && p_dir.x > 0) {
			return 0;
		} else if (p_dir.y < 0 && p_dir.x <= 0) {
			return 1;
		} else if (p_dir.y <= 0 && p_dir.x < 0) {
			return 2;
		} else {
			return 3;
		}
	}

	bool operator()(const PointEntry &p_lhs, const PointEntry &p_rhs) const {
		// Offset quadrant by 2 if flipping.
		const int lhs_q = (quadrant(p_lhs.dir) + (Flip ? 2 : 0)) % 4;
		const int rhs_q = (quadrant(p_rhs.dir) + (Flip ? 2 : 0)) % 4;

		// Sort by quadrant first.
		if (lhs_q < rhs_q) {
			return true;
		} else if (lhs_q > rhs_q) {
			return false;
		} else {
			const real_t ori = p_lhs.dir.cross(p_rhs.dir);

			// If collinear, compare distance, otherwise smaller if RIGHT turn.
			if (ori == 0) {
				return p_lhs.dir.length_squared() < p_rhs.dir.length_squared();
			} else {
				return ori < 0;
			}
		}
	}
};

struct EdgeEntry;

struct EdgeEntryCmp {
	inline bool operator()(const EdgeEntry *p_lhs, const EdgeEntry *p_rhs) const;
};

struct EdgeEntry {
	Vector2 v0, v1;

	RBSet<EdgeEntry *, EdgeEntryCmp>::Element *pos = nullptr;
};

bool EdgeEntryCmp::operator()(const EdgeEntry *p_lhs, const EdgeEntry *p_rhs) const {
	// The rotational sweep processing below guarantee the following:
	// * The target of any edge is counter-clockwise of the source (forward facing).
	// * The target of any edge is never clockwise of the source of any other edge.
	// * The source of any edge is never counter-clockwise of the target of any other edge.
	// * Edges share no points, they do not cross, overlap, or share endpoints.

	const real_t lhs_x0 = p_lhs->v0.x;
	const real_t lhs_y0 = p_lhs->v0.y;

	const real_t lhs_x1 = p_lhs->v1.x;
	const real_t lhs_y1 = p_lhs->v1.y;

	const real_t rhs_x0 = p_rhs->v0.x;
	const real_t rhs_y0 = p_rhs->v0.y;

	const real_t rhs_x1 = p_rhs->v1.x;
	const real_t rhs_y1 = p_rhs->v1.y;

	// The processing is simplified by the fact that edges are either horizontal or vertical.
	const bool lhs_horiz = lhs_y0 == lhs_y1;
	const bool rhs_horiz = rhs_y0 == rhs_y1;

	// If both edges are horizontal or vertical, lhs is closer if it has a smaller absolute y or x value respectively.
	if (lhs_horiz == rhs_horiz) {
		return Math::abs(lhs_horiz ? lhs_y0 : lhs_x0) < Math::abs(lhs_horiz ? rhs_y0 : rhs_x0);
	}

	// The cross product is simplified as the edge is horizontal or vertical.
	const int rhs_dir = ((rhs_horiz ? rhs_x1 : rhs_y1) > (rhs_horiz ? rhs_x0 : rhs_y0)) ? -1 : 1;

	const int lhs_diff_source = SIGN((rhs_horiz ? lhs_y0 : lhs_x0) - (rhs_horiz ? rhs_y0 : rhs_x0));
	const int lhs_diff_target = SIGN((rhs_horiz ? lhs_y1 : lhs_x1) - (rhs_horiz ? rhs_y0 : rhs_x0));

	const int source_ori = rhs_dir * lhs_diff_source * (rhs_horiz ? 1 : -1);
	const int target_ori = rhs_dir * lhs_diff_target * (rhs_horiz ? 1 : -1);

	// lhs is closer than rhs if:
	// * Source or target is to the LEFT of rhs, and the other is not to the RIGHT.
	// * Source and target are on opposite sides, and source of rhs is to the RIGHT of lhs.
	if (source_ori == 0) {
		return target_ori == 1;
	} else {
		if (source_ori == -target_ori) {
			return Math::abs(lhs_horiz ? lhs_y0 : lhs_x0) < Math::abs(lhs_horiz ? rhs_y0 : rhs_x0);
		} else {
			return source_ori == 1;
		}
	}
}

static void star_poly(const Vector<Vector2> &p_points, int p_first, int p_last, LocalVector<Point2i> &p_ranges, LocalVector<Point2i> &p_windows) {
	// The window endpoints.
	const Vector2 &first_p = p_points[p_first];
	const Vector2 &last_p = p_points[p_last];

	// The middle of the window.
	const Vector2 q = (first_p + last_p) / 2;

	// The direction of the window.
	const Vector2 wd = first_p - last_p;

	// Need flipping if positive X-axis lies inside the window, and not on the start direction.
	const bool needs_flip = (wd.y > 0) || (wd.y == 0 && wd.x < 0);

	LocalVector<PointEntry> points;
	points.reserve(p_last - p_first + 1);

	// Track edges that are considered.
	LocalVector<EdgeEntry> valid_edges;
	valid_edges.resize(p_last - p_first);

	// Gather edges that cross the start direction.
	Vector<int> start_edges;

	// If target of first edge is not behind window.
	if (wd.cross(q - p_points[p_first + 1]) >= 0) {
		// Mark first edge as valid
		valid_edges[0].v0 = first_p - q;
		valid_edges[0].v1 = p_points[p_first + 1] - q;

		points.push_back(PointEntry{ valid_edges[0].v0, p_first });
		points.push_back(PointEntry{ valid_edges[0].v1, p_first + 1 });
	}

	// Traverse edges beyond first, checking which are considered.
	for (int i = p_first + 2; i < p_last; ++i) {
		const Vector2 &p0 = p_points[i - 1];
		const Vector2 &p1 = p_points[i];

		// Index of start point indicates horizontal or vertical.
		if (i % 2) {
			// If vertical, then collinear with q if same X coordinate.
			if (p0.x == q.x) {
				// Point on edge that might block, depending on which end is closer.
				const bool on_source = (p0.y < q.y) == (p1.y < p0.y);

				const int first_ind = i + (on_source ? -2 : 1);
				const int second_ind = i + (on_source ? 1 : -2);

				// Not visible if:
				// * Going up and candidate edge is to the right of q
				// * Going down and candidate edge is to the left of q
				// * If edge closer goes one way and edge further goes other way
				if ((p_points[first_ind].x < p0.x) == (p0.y > p1.y) || (p_points[first_ind].x < p0.x) != (p_points[second_ind].x < p0.x)) {
					continue;
				}
			}
			// Otherwise front facing if going down and on the right of q, or going up and on the left of q.
			else if ((p0.x > q.x) == (p1.y < p0.y)) {
				const real_t o = wd.cross(q - p0);

				// If source is to the right of the window it might be a start segment, or hidden.
				if (o < 0) {
					// If target is not to the left, skip segment.
					if (wd.cross(p1 - q) >= 0) {
						continue;
					}

					start_edges.push_back(i - 1);
				} else if (o == 0 && wd.cross(p1 - q) >= 0) {
					continue;
				}

				// Check visibility against closer segment, if segment does not cross X-axis.
				if ((p0.y >= q.y) == (p1.y >= q.y) && p0.y != q.y) {
					// Flip order if top left or bottom right of q.
					const bool flip = (p0.x > q.x) == (p0.y < q.y);

					const Vector2 &other_p = flip ? p_points[i - 2] : p0;
					const Vector2 &further_p = flip ? p1 : p_points[i + 1];

					// Check if further end of this is hidden by other end of edge starting at closer end.
					if ((other_p - q).cross(further_p - q) > 0) {
						continue;
					}
				}
			} else {
				continue;
			}
		} else {
			// If horizontal, then collinear with q if same y coordinate.
			if (p0.y == q.y) {
				// Point on edge that might block, depending on which end is closer.
				const bool on_source = (p0.x > q.x) == (p1.x > p0.x);

				const int first_ind = i + (on_source ? -2 : 1);
				const int second_ind = i + (on_source ? 1 : -2);

				// Not visible if:
				// * Going to the left and candidate edge is above q
				// * Going to the right and candidate edge is below q
				// * If edge closer goes one way and edge further goes other way
				if ((p_points[first_ind].y > p0.y) == (p0.x > p1.x) || (p_points[first_ind].y < p0.y) != (p_points[second_ind].y < p0.y)) {
					continue;
				}
			}
			// Front facing if going left and below q, or right and above q.
			else if ((p0.y < q.y) == (p1.x < p0.x)) {
				const real_t o = wd.cross(q - p0);

				// If source is to the right of the window it might be a start segment, or hidden.
				if (o < 0) {
					// If target is not to the left, skip segment.
					if (wd.cross(q - p1) <= 0) {
						continue;
					}

					start_edges.push_back(i - 1);
				} else if (o == 0 && wd.cross(q - p1) <= 0) {
					continue;
				}

				// Check visibility against closer segment, if segment does not cross Y-axis.
				if ((p0.x >= q.x) == (p1.x >= q.x) && p0.x != q.x) {
					// Flip order if top left or bottom right of q.
					const bool flip = (p0.x > q.x) != (p0.y < q.y);

					const Vector2 &p_a = flip ? p_points[i - 2] : p0;
					const Vector2 &p_b = flip ? p1 : p_points[i + 1];

					// Check if further end of this is hidden by other end of edge starting at closer end.
					if ((p_a - q).cross(p_b - q) > 0) {
						continue;
					}
				}
			} else {
				continue;
			}
		}

		// If edge is not skipped.

		// Mark edge as valid and assign vectors.
		valid_edges[(i - 1) - p_first].v0 = p0 - q;
		valid_edges[(i - 1) - p_first].v1 = p1 - q;

		// If source of this edge is not already on a valid edge, and it is not hidden by window.
		if (wd.cross(q - p0) >= 0 && valid_edges[(i - 2) - p_first].v0 == Vector2()) {
			points.push_back(PointEntry{ valid_edges[(i - 1) - p_first].v0, i - 1 });
		}

		// If target of this edge is not behind window.
		if (wd.cross(q - p1) >= 0) {
			points.push_back(PointEntry{ valid_edges[(i - 1) - p_first].v1, i });
		}
	}

	// If source of last edge is not behind window.
	if (wd.cross(q - p_points[p_last - 1]) >= 0) {
		// Mark edge as valid and assign vectors.
		valid_edges[(p_last - 1) - p_first].v0 = p_points[p_last - 1] - q;
		valid_edges[(p_last - 1) - p_first].v1 = last_p - q;

		// If source of this edge is not already on a valid edge.
		if (valid_edges[(p_last - 2) - p_first].v0 == Vector2()) {
			points.push_back(PointEntry{ valid_edges[(p_last - 1) - p_first].v0, p_last - 1 });
		}

		points.push_back(PointEntry{ valid_edges[(p_last - 1) - p_first].v1, p_last });
	}

	// Sort points, based on flipping.
	if (needs_flip) {
		points.sort_custom<PointEntryCmp<true>>();
	} else {
		points.sort_custom<PointEntryCmp<false>>();
	}

	// Handle collinear points.
	for (uint32_t i = 0; i < points.size() - 1;) {
		const Vector2 first_dir = points[i].dir;

		// Skip if not collinear.
		if (first_dir.cross(points[i + 1].dir) != 0) {
			++i;
			continue;
		}

		// Track if points are in forward or reverse order based on input.
		const bool is_forward = points[i].ind < points[i + 1].ind;

		uint32_t j = i + 2;

		// Iterate until:
		// * Non-collinear point.
		// * The order switches.
		// * There is an edge blocking the following points.
		for (; j < points.size(); ++j) {
			if (first_dir.cross(points[j].dir) != 0 || (points[j - 1].ind < points[j].ind) != is_forward) {
				break;
			} else {
				const Vector2 &other_dir = p_points[points[j].ind + (is_forward ? -1 : 1)] - q;

				// If forward, blocking if on the left, otherwise on the right.
				if (SIGN(first_dir.cross(other_dir)) == (is_forward ? -1 : 1)) {
					break;
				}
			}

			// If the previous and current point make up an edge, and the edge on the closer point on this edge is non-blocking, it can be skipped.
			// This is because there is a vertex closer to this edge that is collinear with q, which makes it invisible.
			if (is_forward && points[j - 1].ind + 1 == points[j].ind && first_dir.cross(p_points[points[j - 1].ind - 1] - q) > 0) {
				valid_edges[(points[j - 1].ind - 1) - p_first].v0 = Vector2();
			} else if (!is_forward && points[j - 1].ind == points[j].ind + 1 && first_dir.cross(p_points[points[j - 1].ind + 1] - q) < 0) {
				valid_edges[points[j - 1].ind - p_first].v0 = Vector2();
			}
		}

		const uint32_t mid_j = j;

		// Iterate while points are collinear.
		for (; j < points.size(); ++j) {
			if (first_dir.cross(points[j].dir) != 0) {
				break;
			}

			// If previous and current make up an edge, skip that edge, as it is invisible.
			if (points[j - 1].ind + 1 == points[j].ind) {
				valid_edges[points[j - 1].ind - p_first].v0 = Vector2();
			} else if (points[j - 1].ind == points[j].ind + 1) {
				valid_edges[points[j].ind - p_first].v0 = Vector2();
			}
		}

		// If not forward, the points need to be rearranged, by putting the points in the first part at the end, in reverse order.
		if (!is_forward) {
			for (uint32_t k = j; i < mid_j && k > i; --k, ++i) {
				SWAP(points[k - 1], points[i]);
			}
		}

		i = j;
	}

	// Set of edges for finding ordering.
	RBSet<EdgeEntry *, EdgeEntryCmp> active_edges;

	// The current closest edge.
	RBSet<EdgeEntry *, EdgeEntryCmp>::Element *front_edge = nullptr;

	if (!start_edges.is_empty()) {
		for (const int &edge : start_edges) {
			// Skip if edge is not considered.
			if (valid_edges[edge - p_first].v0 == Vector2()) {
				continue;
			}

			valid_edges[edge - p_first].pos = active_edges.insert(&valid_edges[edge - p_first]);

			// Update front edge if new edge is front.
			if (valid_edges[edge - p_first].pos->prev() == nullptr) {
				front_edge = valid_edges[edge - p_first].pos;
			}
		}
	}

	// Add start point range to result.
	p_ranges.push_back(Point2i(p_first, p_first));

	for (uint32_t i = 0; i < points.size(); ++i) {
		const int p_i = points[i].ind;
		const Vector2 p_dir = points[i].dir;

		// Check if previous or next edges are valid.
		const bool has_prev = (p_i > p_first) && valid_edges[(p_i - 1) - p_first].v0 != Vector2();
		const bool has_next = (p_i < p_last) && valid_edges[p_i - p_first].v0 != Vector2();

		EdgeEntry *prev = has_prev ? &valid_edges[(p_i - 1) - p_first] : nullptr;
		EdgeEntry *next = has_next ? &valid_edges[p_i - p_first] : nullptr;

		if (has_prev && has_next) {
			// Both edges are present, in this case the previous edge is on the set, and the next is not.
			ERR_FAIL_COND(!(prev->pos && !next->pos));

			// When two edges share an endpoint they will always be in the position in the set, so simply update this.
			next->pos = prev->pos;

			// Change which edge is pointed to, to avoid erasing and re-inserting.
			next->pos->get() = next;

			// If edge is not front edge, skip.
			if (next->pos->prev() != nullptr) {
				continue;
			}
		} else if (has_prev) {
			// Previous edge is present, the previous edge must be on the set.
			ERR_FAIL_COND(prev->pos == nullptr);

			// Check if edge was front edge.
			const bool was_front = (prev->pos->prev() == nullptr);

			// If edge was front edge, update front edge.
			if (was_front) {
				front_edge = prev->pos->next();
			}

			// Erase edge.
			active_edges.erase(prev->pos);

			prev->pos = nullptr;

			// If edge was not front, skip.
			if (!was_front) {
				continue;
			}
		} else if (has_next) {
			// Next edge is present, the next edge must not be on the set.
			ERR_FAIL_COND(next->pos);

			// Special case if edge is collinear with q.
			if (p_dir[p_i % 2] == 0) {
				// Check if edge after next is present.
				const bool has_next_next = (p_i + 1 < p_last) && valid_edges[(p_i + 1) - p_first].v0 != Vector2();

				EdgeEntry *next_next = has_next_next ? &valid_edges[(p_i + 1) - p_first] : nullptr;

				// Skip this point.
				++i;

				const int next_i = points[i].ind;
				const Vector2 next_dir = points[i].dir;

				// If edge after next is valid.
				if (next_next) {
					// Insert edge after next.
					next_next->pos = active_edges.insert(next_next);

					// If new front edge, update front edge, otherwise skip.
					if (next_next->pos->prev() == nullptr) {
						front_edge = next_next->pos;
					} else {
						continue;
					}
				}
				// Otherwise skip if there is a front edge, and it is closer than next edge.
				else if (front_edge && Math::abs(p_dir[1 - (p_i % 2)]) >= Math::abs(front_edge->get()->v0[1 - (p_i % 2)])) {
					continue;
				}

				// Add the endpoints of next edge to the ranges.
				if (p_ranges[p_ranges.size() - 1].y + 1 < p_i) {
					p_windows.push_back(Point2i(p_ranges[p_ranges.size() - 1].y, p_i));
					p_ranges.push_back(Point2i(p_i, next_i));
				} else {
					p_ranges[p_ranges.size() - 1].y = next_i;
				}

				continue;
			} else {
				// Insert edge.
				next->pos = active_edges.insert(next);

				// If new front edge, update front edge, otherwise skip.
				if (next->pos->prev() == nullptr) {
					front_edge = next->pos;
				} else {
					continue;
				}
			}
		} else {
			continue;
		}

		// If we did not skip, the current point should be inserted into result.
		if (p_ranges[p_ranges.size() - 1].y + 1 < p_i) {
			p_windows.push_back(Point2i(p_ranges[p_ranges.size() - 1].y, p_i));
			p_ranges.push_back(Point2i(p_i, p_i));
		} else {
			p_ranges[p_ranges.size() - 1].y = p_i;
		}
	}

	// Add last point if not present.
	if (p_ranges[p_ranges.size() - 1].y != p_last) {
		if (p_ranges[p_ranges.size() - 1].y + 1 < p_last) {
			p_windows.push_back(Point2i(p_ranges[p_ranges.size() - 1].y, p_last));
			p_ranges.push_back(Point2i(p_last, p_last));
		} else {
			p_ranges[p_ranges.size() - 1].y = p_last;
		}
	}
}

#define SET_BIT(bits, ind) bits[(ind) / 8] |= (1 << ((ind) % 8))
#define CLEAR_BIT(bits, ind) bits[(ind) / 8] &= ~(1 << ((ind) % 8))
#define TEST_BIT(bits, ind) (bits[(ind) / 8] & (1 << ((ind) % 8)))

static int rdp_plain(const Vector<Vector2> &p_points, int p_first, int p_last, float p_eps_2, LocalVector<uint8_t> &p_selected, bool p_total) {
	if (p_last - p_first < 2) {
		return 0;
	}

	// Track furthest point from the line.
	float dist = -1.0;

	int ind = 0;

	const Vector2 v = p_points[p_last] - p_points[p_first];

	// Iterate over points, ignoring first and last.
	for (int i = p_first + 1; i < p_last; ++i) {
		// Get distance from line, as the cross product with the line.
		const float cur_dist = Math::abs((p_points[i] - p_points[p_first]).cross(v));

		if (dist < cur_dist) {
			dist = cur_dist;
			ind = i;
		}
	}

	// If over threshold, recurse.
	if (dist * dist / v.length_squared() > p_eps_2) {
		SET_BIT(p_selected, ind);

		return rdp_plain(p_points, p_first, ind, p_eps_2, p_selected, false) +
				rdp_plain(p_points, ind, p_last, p_eps_2, p_selected, false) + 1;
	} else {
		// If total, keep furthest point, to keep a polygon.
		if (p_total) {
			SET_BIT(p_selected, ind);

			return 1;
		}

		return 0;
	}
}

static int rdp(const Vector<Vector2> &p_points, int p_first, int p_last, float p_eps_2, LocalVector<uint8_t> &p_selected, LocalVector<Vector3i> &p_crossings, Rect2 &p_bounds, bool p_ensure_poly = false) {
	if (p_last - p_first < 2) {
		return 0;
	}

	// Track furthest point from the line, also track point furthest inside for crossing checks.
	float dist = -1.0;
	float dist_inner = 0.0;

	int ind = 0;
	int ind_inner = 0;

	const Vector2 v = p_points[p_last] - p_points[p_first];

	// Iterate over points, ignoring first and last.
	for (int i = p_first + 1; i < p_last; ++i) {
		// Get signed distance from line, as the cross product with the line.
		const float cur_dist = (p_points[i] - p_points[p_first]).cross(v);

		// If distance is non-negative the point is inside.
		if (cur_dist >= 0) {
			if (dist_inner < cur_dist) {
				dist_inner = cur_dist;
				ind_inner = i;
			}

			if (dist < cur_dist) {
				dist = cur_dist;
				ind = i;
			}
		} else if (dist < -cur_dist) {
			dist = -cur_dist;
			ind = i;
		}
	}

	// If over threshold, recurse.
	if (dist * dist / v.length_squared() > p_eps_2) {
		SET_BIT(p_selected, ind);
		p_bounds.expand_to(p_points[ind]);

		return rdp(p_points, p_first, ind, p_eps_2, p_selected, p_crossings, p_bounds) +
				rdp(p_points, ind, p_last, p_eps_2, p_selected, p_crossings, p_bounds) + 1;
	} else {
		if (p_ensure_poly) {
			SET_BIT(p_selected, ind);
			p_bounds.expand_to(p_points[ind]);

			return 1;
		} else if (ind_inner) {
			// Mark crossing for furthest inner point.
			p_crossings.push_back(Vector3i(p_first, p_last, ind_inner));
		}

		return 0;
	}
}

static Vector<Vector2> reduce_2(const Vector<Vector2> &p_points, const Rect2i &p_rect, float p_epsilon) {
	// Do nothing if epsilon is non-positive.
	if (!(p_epsilon > 0.0)) {
		return p_points;
	}

	LocalVector<LocalVector<Point2i>> ranges;
	LocalVector<Point2i> windows;

	windows.push_back(Point2i(0, p_points.size() - 1));

	while (!windows.is_empty()) {
		const Point2i window = windows[windows.size() - 1];
		windows.resize(windows.size() - 1);

		if (window.y - window.x <= 3) {
			ranges.push_back({ window });
		} else {
			ranges.resize(ranges.size() + 1);

			star_poly(p_points, window.x, window.y, ranges[ranges.size() - 1], windows);
		}
	}

	print_line("{");

	for (uint32_t k = 0; k < ranges.size(); ++k) {
		StringBuilder builder;
		builder += " { ";

		for (uint32_t l = 0; l < ranges[k].size(); ++l) {
			builder += (String)ranges[k][l];
			builder += " ";
		}

		builder += "}";
		print_line(builder.as_string());
	}

	print_line("}");

	LocalVector<uint8_t> selected;
	selected.resize(((p_points.size() - 1) / 8) + 1);
	memset(selected.ptr(), 0, selected.size());

	SET_BIT(selected, 0);
	SET_BIT(selected, p_points.size() - 1);

	if (ranges.size() > 1) {
		LocalVector<LocalVector<Vector3i>> crossings;
		LocalVector<LocalVector<Rect2>> bounds;

		crossings.resize(ranges.size());
		bounds.resize(ranges.size());

		int num_points = 1;

		for (uint32_t k = 0; k < ranges.size(); ++k) {
			bounds[k].resize(ranges[k].size());

			const int &first_i = ranges[k][0].x;
			const int &last_i = ranges[k][ranges[k].size() - 1].y;

			const Vector2 &first_p = p_points[first_i];
			const Vector2 &last_p = p_points[last_i];
			const Vector2 v = last_p - first_p;

			for (uint32_t l = 0; l < ranges[k].size(); ++l) {
				if (ranges[k][l].y - ranges[k][l].x >= 1) {
					const int &i0 = ranges[k][l].x;
					const int &i1 = ranges[k][l].y;

					const Vector2 &p0 = p_points[i0];
					const Vector2 &p1 = p_points[i1];

					SET_BIT(selected, i0);
					bounds[k][l] = Rect2(p0, Vector2());

					bounds[k][l].expand_to(p1);

					// Range needs special treatment if endpoints arent the ends of the total range, but are on the line of the ends.
					const bool ensure_poly = (i0 != first_i || i1 != last_i) && (i0 == first_i || v.cross(p0 - first_p) == 0) && (i1 == last_i || v.cross(p1 - first_p) == 0);

					num_points += rdp(p_points, i0, i1, p_epsilon * p_epsilon, selected, crossings[k], bounds[k][l], ensure_poly) + 1;
				}
			}
		}

		// TODO fix intersections between regions.

		// Fix collinear points.
		int prev_p = 1, prev_prev_p = 0;

		// Find first selected vertex after start.
		for (; prev_p < p_points.size() - 1; ++prev_p) {
			if (TEST_BIT(selected, prev_p)) {
				break;
			}
		}

		for (int k = prev_p + 1; k < p_points.size() - 1; ++k) {
			if (TEST_BIT(selected, k)) {
				const Vector2 &p0 = p_points[prev_prev_p];
				const Vector2 &p1 = p_points[prev_p];
				const Vector2 &p2 = p_points[k];

				if ((p1 - p0).cross(p2 - p0) == 0) {
					CLEAR_BIT(selected, prev_p);
					num_points -= 1;
				} else {
					prev_prev_p = prev_p;
				}

				prev_p = k;
			}
		}

		Vector<Vector2> polygon;
		polygon.resize(num_points);

		Vector2 *ptr = polygon.ptrw();

		{
			StringBuilder builder;
			builder += "C: { ";

			for (uint32_t k = 0; k < crossings.size(); ++k) {
				builder += "{ ";

				for (uint32_t l = 0; l < crossings[k].size(); ++l) {
					builder += (String)crossings[k][l];
					builder += " ";
				}

				builder += "} ";
			}

			builder += "}";

			print_line(builder.as_string());
		}

		{
			StringBuilder builder;
			builder += "B: { ";

			for (uint32_t k = 0; k < bounds.size(); ++k) {
				builder += "{ ";

				for (uint32_t l = 0; l < bounds[k].size(); ++l) {
					builder += (String)bounds[k][l];
					builder += " ";
				}

				builder += "} ";
			}

			builder += "}";

			print_line(builder.as_string());
		}

		{
			StringBuilder builder;
			builder += "P: ";
			builder += String::num_int64(num_points);
			builder += " { ";

			for (int k = 0; k < p_points.size(); ++k) {
				if (TEST_BIT(selected, k)) {
					builder += String::num_int64(k);
					builder += " ";
					*ptr++ = p_points[k];
				}
			}

			builder += "}";

			print_line(builder.as_string());
		}

		return polygon;
	} else {
		int num_points = 2 + rdp_plain(p_points, 0, p_points.size() - 1, p_epsilon * p_epsilon, selected, true);

		Vector<Vector2> polygon;

		{
			StringBuilder builder;
			builder += "P: ";
			builder += String::num_int64(num_points);
			builder += " { ";

			for (int k = 0; k < p_points.size(); ++k) {
				if (TEST_BIT(selected, k)) {
					builder += String::num_int64(k);
					builder += " ";
					polygon.push_back(p_points[k]);
				}
			}

			builder += "}";

			print_line(builder.as_string());
		}

		return polygon;
	}
}

struct FillBitsStackEntry {
	Point2i pos;
	int i = 0;
	int j = 0;
};

static void fill_bits(const BitMap *p_src, Ref<BitMap> &p_map, const Point2i &p_pos, const Rect2i &rect) {
	// Using a custom stack to work iteratively to avoid stack overflow on big bitmaps.
	Vector<FillBitsStackEntry> stack;
	// Tracking size since we won't be shrinking the stack vector.
	int stack_size = 0;

	Point2i pos = p_pos;
	int next_i = 0;
	int next_j = 0;

	bool reenter = true;
	bool popped = false;
	do {
		if (reenter) {
			next_i = pos.x - 1;
			next_j = pos.y - 1;
			reenter = false;
		}

		for (int i = next_i; i <= pos.x + 1; i++) {
			for (int j = next_j; j <= pos.y + 1; j++) {
				if (popped) {
					// The next loop over j must start normally.
					next_j = pos.y - 1;
					popped = false;
					// Skip because an iteration was already executed with current counter values.
					continue;
				}

				if (i < rect.position.x || i >= rect.position.x + rect.size.x) {
					continue;
				}
				if (j < rect.position.y || j >= rect.position.y + rect.size.y) {
					continue;
				}

				if (p_map->get_bit(i, j)) {
					continue;

				} else if (p_src->get_bit(i, j)) {
					p_map->set_bit(i, j, true);

					FillBitsStackEntry se = { pos, i, j };
					stack.resize(MAX(stack_size + 1, stack.size()));
					stack.set(stack_size, se);
					stack_size++;

					pos = Point2i(i, j);
					reenter = true;
					break;
				}
			}
			if (reenter) {
				break;
			}
		}
		if (!reenter) {
			if (stack_size) {
				FillBitsStackEntry se = stack.get(stack_size - 1);
				stack_size--;
				pos = se.pos;
				next_i = se.i;
				next_j = se.j;
				popped = true;
			}
		}
	} while (reenter || popped);
}

Vector<Vector<Vector2>> BitMap::clip_opaque_to_polygons(const Rect2i &p_rect, float p_epsilon) const {
	Rect2i r = Rect2i(0, 0, width, height).intersection(p_rect);

	Point2i from;
	Ref<BitMap> fill;
	fill.instantiate();
	fill->create(get_size());

	Vector<Vector<Vector2>> polygons;
	for (int i = r.position.y; i < r.position.y + r.size.height; i++) {
		for (int j = r.position.x; j < r.position.x + r.size.width; j++) {
			if (!fill->get_bit(j, i) && get_bit(j, i)) {
				fill_bits(this, fill, Point2i(j, i), r);

				for (Vector<Vector2> polygon : _march_square(r, Point2i(j, i))) {
					polygon = reduce_2(polygon, r, p_epsilon);

					if (polygon.size() < 3) {
						print_verbose("Invalid polygon, skipped");
						continue;
					}

					polygons.push_back(polygon);
				}
			}
		}
	}

	return polygons;
}

void BitMap::grow_mask(int p_pixels, const Rect2i &p_rect) {
	if (p_pixels == 0) {
		return;
	}

	bool bit_value = p_pixels > 0;
	p_pixels = Math::abs(p_pixels);

	Rect2i r = Rect2i(0, 0, width, height).intersection(p_rect);

	Ref<BitMap> copy;
	copy.instantiate();
	copy->create(get_size());
	copy->bitmask = bitmask;

	for (int i = r.position.y; i < r.position.y + r.size.height; i++) {
		for (int j = r.position.x; j < r.position.x + r.size.width; j++) {
			if (bit_value == get_bit(j, i)) {
				continue;
			}

			bool found = false;

			for (int y = i - p_pixels; y <= i + p_pixels; y++) {
				for (int x = j - p_pixels; x <= j + p_pixels; x++) {
					bool outside = false;

					if ((x < p_rect.position.x) || (x >= p_rect.position.x + p_rect.size.x) || (y < p_rect.position.y) || (y >= p_rect.position.y + p_rect.size.y)) {
						// Outside of rectangle counts as bit not set.
						if (!bit_value) {
							outside = true;
						} else {
							continue;
						}
					}

					float d = Point2(j, i).distance_to(Point2(x, y)) - CMP_EPSILON;
					if (d > p_pixels) {
						continue;
					}

					if (outside || (bit_value == copy->get_bit(x, y))) {
						found = true;
						break;
					}
				}
				if (found) {
					break;
				}
			}

			if (found) {
				set_bit(j, i, bit_value);
			}
		}
	}
}

void BitMap::shrink_mask(int p_pixels, const Rect2i &p_rect) {
	grow_mask(-p_pixels, p_rect);
}

TypedArray<PackedVector2Array> BitMap::_opaque_to_polygons_bind(const Rect2i &p_rect, float p_epsilon) const {
	Vector<Vector<Vector2>> result = clip_opaque_to_polygons(p_rect, p_epsilon);

	// Convert result to bindable types.

	TypedArray<PackedVector2Array> result_array;
	result_array.resize(result.size());
	for (int i = 0; i < result.size(); i++) {
		const Vector<Vector2> &polygon = result[i];

		PackedVector2Array polygon_array;
		polygon_array.resize(polygon.size());

		{
			Vector2 *w = polygon_array.ptrw();
			for (int j = 0; j < polygon.size(); j++) {
				w[j] = polygon[j];
			}
		}

		result_array[i] = polygon_array;
	}

	return result_array;
}

void BitMap::resize(const Size2i &p_new_size) {
	ERR_FAIL_COND(p_new_size.width < 0 || p_new_size.height < 0);
	if (p_new_size == get_size()) {
		return;
	}

	Ref<BitMap> new_bitmap;
	new_bitmap.instantiate();
	new_bitmap->create(p_new_size);
	// also allow for upscaling
	int lw = (width == 0) ? 0 : p_new_size.width;
	int lh = (height == 0) ? 0 : p_new_size.height;

	float scale_x = ((float)width / p_new_size.width);
	float scale_y = ((float)height / p_new_size.height);
	for (int x = 0; x < lw; x++) {
		for (int y = 0; y < lh; y++) {
			bool new_bit = get_bit(x * scale_x, y * scale_y);
			new_bitmap->set_bit(x, y, new_bit);
		}
	}

	width = new_bitmap->width;
	height = new_bitmap->height;
	bitmask = new_bitmap->bitmask;
}

Ref<Image> BitMap::convert_to_image() const {
	Ref<Image> image = Image::create_empty(width, height, false, Image::FORMAT_L8);

	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			image->set_pixel(i, j, get_bit(i, j) ? Color(1, 1, 1) : Color(0, 0, 0));
		}
	}

	return image;
}

void BitMap::blit(const Vector2i &p_pos, const Ref<BitMap> &p_bitmap) {
	ERR_FAIL_COND_MSG(p_bitmap.is_null(), "It's not a reference to a valid BitMap object.");

	int x = p_pos.x;
	int y = p_pos.y;
	int w = p_bitmap->get_size().width;
	int h = p_bitmap->get_size().height;

	for (int i = 0; i < w; i++) {
		for (int j = 0; j < h; j++) {
			int px = x + i;
			int py = y + j;
			if (px < 0 || px >= width) {
				continue;
			}
			if (py < 0 || py >= height) {
				continue;
			}
			if (p_bitmap->get_bit(i, j)) {
				set_bit(px, py, true);
			}
		}
	}
}

void BitMap::_bind_methods() {
	ClassDB::bind_method(D_METHOD("create", "size"), &BitMap::create);
	ClassDB::bind_method(D_METHOD("create_from_image_alpha", "image", "threshold"), &BitMap::create_from_image_alpha, DEFVAL(0.1));

	ClassDB::bind_method(D_METHOD("set_bitv", "position", "bit"), &BitMap::set_bitv);
	ClassDB::bind_method(D_METHOD("set_bit", "x", "y", "bit"), &BitMap::set_bit);
	ClassDB::bind_method(D_METHOD("get_bitv", "position"), &BitMap::get_bitv);
	ClassDB::bind_method(D_METHOD("get_bit", "x", "y"), &BitMap::get_bit);

	ClassDB::bind_method(D_METHOD("set_bit_rect", "rect", "bit"), &BitMap::set_bit_rect);
	ClassDB::bind_method(D_METHOD("get_true_bit_count"), &BitMap::get_true_bit_count);

	ClassDB::bind_method(D_METHOD("get_size"), &BitMap::get_size);
	ClassDB::bind_method(D_METHOD("resize", "new_size"), &BitMap::resize);

	ClassDB::bind_method(D_METHOD("_set_data", "data"), &BitMap::_set_data);
	ClassDB::bind_method(D_METHOD("_get_data"), &BitMap::_get_data);

	ClassDB::bind_method(D_METHOD("grow_mask", "pixels", "rect"), &BitMap::grow_mask);
	ClassDB::bind_method(D_METHOD("convert_to_image"), &BitMap::convert_to_image);
	ClassDB::bind_method(D_METHOD("opaque_to_polygons", "rect", "epsilon"), &BitMap::_opaque_to_polygons_bind, DEFVAL(2.0));

	ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "data", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_INTERNAL), "_set_data", "_get_data");
}

BitMap::BitMap() {}
