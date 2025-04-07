#include <iostream>
#include <cmath>
#include <exception>
#include <array>
#include <vector>
#include <limits>

#include "ee4308_core/core.hpp"

#pragma once
namespace ee4308
{
    // A line algorithm that 
    class RayTracer
    {
    private:
        std::array<int, 2> abs_dif_;
        std::array<int, 2> err_;
        std::array<int, 2> from_;
        std::array<int, 2> to_;
        std::array<int, 2> err_dif_;
        std::array<int, 2> vert_;
        std::array<int, 2> sgn_;
        std::array<int, 2> err_final_;
        double total_length_ = NAN;

    public:
        RayTracer() {};
        /**
         * see init().
         */
        RayTracer(const int from_mx, const int from_my,
                   const int to_mx, const int to_my)
        {
            init(from_mx, from_my, to_mx, to_my);
        }

        /**
         * Initializes the ray tracer. Returns true if the ray has already reached its destination (i.e. start and goal in the same grid cell).
         * The coordinates of the center of a grid cell are (A+0.5, B+0.5), where A and B are integers.
         * (A, B) are the coordinates of the bottom-left corner (grid vertex) of the cell.
         * @param from_mx {double} The grid "x" coordinates where the ray begins.
         * @param from_my {double} The grid "y" coordinates where the ray begins.
         * @param to_mx {double} The grid "x" coordinates where the ray ends.
         * @param to_my {double} The grid "y" coordinates where the ray ends.
         */
        void init(const int from_mx, const int from_my,
                  const int to_mx, const int to_my)
        {
            from_ = {from_mx, from_my};
            to_ = {to_mx, to_my};
            std::array<int, 2> dif = {to_mx - from_mx, to_my - from_my};
            abs_dif_ = {abs(dif[0]), abs(dif[1])};
            sgn_ = {sgn(dif[0]), sgn(dif[1])};

            err_dif_ = {2, 2};
            err_ = {1, 1};
            err_final_ = {2 * abs_dif_[0], 2 * abs_dif_[1]};
            total_length_ = std::hypot((double)abs_dif_[0], (double)abs_dif_[1]);
            for (int d = 0; d < 2; ++d)
            {
                if (dif[d] > 0)
                    vert_[d] = from_[d]; 
                else if (dif[d] < 0)
                    vert_[d] = from_[d] + 1; 
                else
                    vert_[d] = from_[d]; 
            }
        }

        /**
         * Moves the position to the next root vertex.
         * The algorithm stops at every intersection of the ray with the horizontal or vertical grid lines.
         *
         * Suppose the unit direction is (dx, dy), and the current root vertex is (x,y).
         * If the closest grid line to intersect is the vertical line at x, the root vertex changes from (x, y) to (x+dx, y) in this call to next().
         * If the closest grid line is the horizontal line at y, the root vertex changes from (x, y) to (x, y+dy).
         */
        void next()
        {
            auto getNext = [this](const int &d) -> void
            {
                vert_[d] += sgn_[d];
                err_[d] += err_dif_[d];
            };

            int norm_len_x = err_[0] * abs_dif_[1];
            int norm_len_y = err_[1] * abs_dif_[0];

            if (norm_len_x < norm_len_y)
                getNext(0); // x crossed a grid line before y crossed a grid line
            else if (norm_len_y < norm_len_x)
                getNext(1); // y crossed a grid line before x crossed a grid line
            else
            { // x and y crossed grid line at same location
                getNext(0);
                getNext(1);
            }
        }

        /**
         * Returns the bottom-left coordinates of the cell(s) in the direction in front of the grid vertex.
         * This functions returns (A, B) for a cell with center at (A + 0.5, B + 0.5). A and B are integers.
         * If the direction is cardinal, the two cells diagonally in front will be returned.
         * If the direction is diagonal, only one cell is returned.
         */
        std::array<int, 2> frontCell() const
        {
            int dx = sgn_[0] >= 0 ? 0 : -1;
            int dy = sgn_[1] >= 0 ? 0 : -1;

            return {vert_[0] + dx, vert_[1] + dy};
        }

        /**
         * Returns true when the ray has reached the end.
         */
        bool reached() const { return err_[0] >= err_final_[0] && err_[1] >= err_final_[1]; }

        /**
         * Returns the total length of the ray.
         */
        const double &totalLength() const { return total_length_; }

        /**
         * Returns the length currently traversed by the ray.
         * Can be longer than totalLength() if the coordinate to stop is not at a vertex.
         */
        double length() const
        {
            double norm_len_x = 0.5 * err_[0] / abs_dif_[0]; // different from the norm_len in next()
            double norm_len_y = 0.5 * err_[1] / abs_dif_[1]; // different from the norm_len in next()
            if (norm_len_x < norm_len_y)
                return total_length_ * norm_len_x;
            else
                return total_length_ * norm_len_y;
        }
    };

    // /**
    //  * A general purpose 2D symmetric ray tracer that does not consider rays that are colinear to grid lines.
    //  * Returns all intermediate cells unlike Bresenham or DDA.
    //  */
    // class RayTracerGeneral
    // {

    // private:
    //     std::array<double, 2> dif_;
    //     std::array<double, 2> err_;
    //     std::array<double, 2> from_;
    //     std::array<double, 2> to_;
    //     std::array<double, 2> err_dif_;
    //     std::array<int, 2> vert_;
    //     std::array<int, 2> sgn_;
    //     const double REACHED_THRES = 1 - ee4308::THRES;
    //     double total_length_ = NAN;

    // public:
    //     RayTracerGeneral() {};
    //     RayTracerGeneral(const double from_mx, const double from_my,
    //               const double to_mx, const double to_my)
    //     {
    //         init(from_mx, from_my, to_mx, to_my);
    //     }

    //     /**
    //      * Initializes the ray tracer. Returns true if the ray has already reached its destination (i.e. start and goal in the same grid cell).
    //      * The coordinates of the center of a grid cell are (A+0.5, B+0.5), where A and B are integers.
    //      * (A, B) are the coordinates of the bottom-left corner (grid vertex) of the cell.
    //      * @param from_mx {double} The grid "x" coordinates where the ray begins.
    //      * @param from_my {double} The grid "y" coordinates where the ray begins.
    //      * @param to_mx {double} The grid "x" coordinates where the ray ends.
    //      * @param to_my {double} The grid "y" coordinates where the ray ends.
    //      */
    //     void init(const double from_mx, const double from_my,
    //               const double to_mx, const double to_my)
    //     {
    //         from_ = {from_mx, from_my};
    //         to_ = {to_mx, to_my};
    //         dif_ = {to_mx - from_mx, to_my - from_my};
    //         total_length_ = 0;
    //         for (int d = 0; d < 2; ++d)
    //         {
    //             if (dif_[d] > THRES)
    //             { // dif_[d] is positive.
    //                 vert_[d] = std::floor(from_[d]);
    //                 sgn_[d] = 1;
    //                 err_dif_[d] = double(sgn_[d]) / dif_[d];
    //                 err_[d] = (std::ceil(from_[d]) - from_[d]) * err_dif_[d];
    //             }
    //             else if (dif_[d] < -THRES)
    //             { // dif_[d] is negative
    //                 vert_[d] = std::ceil(from_[d]);
    //                 sgn_[d] = -1;
    //                 err_dif_[d] = double(sgn_[d]) / dif_[d];
    //                 err_[d] = (from_[d] - std::floor(from_[d])) * err_dif_[d];
    //             }
    //             else
    //             { // dif_[d] is close to zero.
    //                 vert_[d] = std::floor(from_[d]);
    //                 sgn_[d] = 0;
    //                 err_dif_[d] = INFINITY;
    //                 dif_[d] = 0;
    //                 err_[d] = INFINITY;
    //             }
    //             total_length_ += dif_[d] * dif_[d];
    //         }

    //         total_length_ = std::sqrt(total_length_);
    //     }

    //     /**
    //      * Moves the position to the next root vertex.
    //      * The algorithm stops at every intersection of the ray with the horizontal or vertical grid lines.
    //      *
    //      * Suppose the unit direction is (dx, dy), and the current root vertex is (x,y).
    //      * If the closest grid line to intersect is the vertical line at x, the root vertex changes from (x, y) to (x+dx, y) in this call to next().
    //      * If the closest grid line is the horizontal line at y, the root vertex changes from (x, y) to (x, y+dy).
    //      */
    //     void next()
    //     {
    //         auto getNext = [this](const int &d) -> void
    //         {
    //             vert_[d] += sgn_[d];
    //             err_[d] += err_dif_[d];
    //         };

    //         if (err_[0] < err_[1] - THRES)
    //             getNext(0); // x crossed a grid line before y crossed a grid line
    //         else if (err_[1] < err_[0] - THRES)
    //             getNext(1); // y crossed a grid line before x crossed a grid line
    //         else
    //         { // x and y crossed grid line at same location
    //             getNext(0);
    //             getNext(1);
    //         }
    //     }

    //     /**
    //      * Returns the bottom-left coordinates of the cell(s) in the direction in front of the grid vertex.
    //      * This functions returns (A, B) for a cell with center at (A + 0.5, B + 0.5). A and B are integers.
    //      * If the direction is cardinal, the two cells diagonally in front will be returned.
    //      * If the direction is diagonal, only one cell is returned.
    //      */
    //     std::array<int, 2> frontCell() const
    //     {
    //         int dx = sgn_[0] >= 0 ? 0 : -1;
    //         int dy = sgn_[1] >= 0 ? 0 : -1;

    //         return {vert_[0] + dx, vert_[1] + dy};
    //     }

    //     /**
    //      * Returns true when the ray has reached the end.
    //      */
    //     bool reached() const { return err_[0] >= 1 && err_[1] >= 1; }

    //     /**
    //      * Returns the total length of the ray.
    //      */
    //     const double &totalLength() const { return total_length_; }

    //     /**
    //      * Returns the length currently traversed by the ray.
    //      * Can be longer than totalLength() if the coordinate to stop is not at a vertex.
    //      */
    //     double length() const { return totalLength() * std::min(err_[0], err_[1]); }
    // };

}