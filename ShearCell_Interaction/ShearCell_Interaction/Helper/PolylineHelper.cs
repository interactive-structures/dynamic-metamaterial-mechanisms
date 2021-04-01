using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Windows;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;

namespace ShearCell_Interaction.Helper
{
    public static class PolylineHelper
    {
        public const double DIRECTION_THRESHOLD = 1.0 / 57.2958;

        public static List<Vector> SamplePolygon(List<Point> input, int numberSamples, bool areScreenCoordinates = true)
        {
            PathFigure pathFigure = new PathFigure
            {
                IsClosed = false,
                StartPoint = input[0]
            };

            for (var i = 1; i < input.Count; i++)
            {
                var point = input[i];
                pathFigure.Segments.Add(new LineSegment(point, true));
            }

            pathFigure.Segments.Add(new LineSegment(input[input.Count - 1], true));

            var pathGeometry = new PathGeometry();
            pathGeometry.Figures.Add(pathFigure);

            return SamplePolygon(pathGeometry, numberSamples, Matrix.Identity, areScreenCoordinates);
        }

        public static List<Vector> SamplePolygon(PathGeometry pathGeometry, int numberSamples, Matrix transform, bool areScreenCoordinates = true)
        {
            var gridPathPoints = new List<Vector>();

            var translate = new Vector(transform.OffsetX, transform.OffsetY);

            for (int i = 0; i < numberSamples; i++)
            {
                var progress = 1.0 / (numberSamples - 1) * i;

                Point pointOnPath;
                Point tangent;
                pathGeometry.GetPointAtFractionLength(progress, out pointOnPath, out tangent);

                var fractionPoint = new Vector(pointOnPath.X, pointOnPath.Y);
                var transformedPoint = transform.Transform(fractionPoint);
                transformedPoint = Vector.Add(transformedPoint, translate);

                if (areScreenCoordinates)
                    transformedPoint = CoordinateConverter.ConvertScreenToGlobalCellCoordinates(transformedPoint);

                gridPathPoints.Add(transformedPoint);
            }

            return gridPathPoints;
        }

        public static void ResampleStrokeSimple(Stroke stroke, int samplingFactor = 5)
        {
            var points = stroke.StylusPoints;

            var newPoints = new StylusPointCollection();

            for (int i = 0; i < points.Count; i += samplingFactor)
            {
                newPoints.Add(points[i]);
            }

            newPoints.Add(points[points.Count - 1]);

            stroke.StylusPoints = newPoints;
        }

        public static double PathLength(List<Vector> input)
        {
            double length = 0;

            for (int i = 0; i < input.Count - 1; i++)
            {
                var current = input[i];
                var next = input[i + 1];
                length += Math.Abs(Vector.Subtract(next, current).Length);
            }

            return length;
        }

        public static double PathLength(List<Point> input)
        {
            double length = 0;

            for (int i = 0; i < input.Count - 1; i++)
            {
                var current = input[i];
                var next = input[i + 1];
                length += Math.Abs(Point.Subtract(next, current).Length);
            }

            return length;
        }


        public static double PathLength(PointCollection input, bool convertToCellCoords = false)
        {
            double length = 0;

            for (int i = 0; i < input.Count - 1; i++)
            {
                var current = input[i];
                var next = input[i + 1];

                if (convertToCellCoords)
                {
                    current = CoordinateConverter.ConvertScreenToGlobalCellCoordinates(current);
                    next = CoordinateConverter.ConvertScreenToGlobalCellCoordinates(next);
                }

                length += Math.Abs(Point.Subtract(next, current).Length);
            }

            return length;
        }


        public static double SquaredDistance(List<Point> pointsA, List<Point> pointsB)
        {
            if (pointsA.Count != pointsB.Count)
                throw new Exception($"Number of points do not match. Points A count is {pointsA.Count}; Points B count is {pointsB.Count}");

            var squaredDistance = 0.0;

            for (var i = 0; i < pointsA.Count; i++)
            {
                var pointA = pointsA[i];
                var pointB = pointsB[i];

                squaredDistance += Math.Pow(pointA.X - pointB.X, 2) + Math.Pow(pointA.Y - pointB.Y, 2);
            }

            return squaredDistance;
        }
        public static double SquaredDistance(List<Vector> pointsA, List<Vector> pointsB)
        {
            if (pointsA.Count != pointsB.Count)
                throw new Exception($"Number of points do not match. Points A count is {pointsA.Count}; Points B count is {pointsB.Count}");

            var squaredDistance = 0.0;

            for (var i = 0; i < pointsA.Count; i++)
            {
                var pointA = pointsA[i];
                var pointB = pointsB[i];

                squaredDistance += Math.Pow(pointA.X - pointB.X, 2) + Math.Pow(pointA.Y - pointB.Y, 2);
            }

            return squaredDistance;
        }

        public static double Distance(List<Point> pointsA, List<Point> pointsB)
        {
            return Math.Sqrt(SquaredDistance(pointsA, pointsB));
        }

        public static double Distance(List<Vector> pointsA, List<Vector> pointsB)
        {
            return Math.Sqrt(SquaredDistance(pointsA, pointsB));
        }

        public static int NumInflectionPoints(List<Vector> points, int samplingFactor = 5)
        {
            var directions = new List<double>();
            var segmentLengths = new List<double>();

            var resampledPoints = new List<Vector>();

            for (int i = 0; i < points.Count; i += samplingFactor)
                resampledPoints.Add(points[i]);

            if (points.Count % samplingFactor > 2 && points.Count % samplingFactor < 8)
                resampledPoints.Add(points[points.Count - 1]);

            for (int i = 0; i < resampledPoints.Count - 1; i++)
            {
                var current = resampledPoints[i];
                var next = resampledPoints[i + 1];

                var direction = Math.Atan2(next.Y - current.Y, next.X - current.X);
                direction = Math.Abs(direction) < DIRECTION_THRESHOLD ? 0 : direction;

                directions.Add(direction);

                while (i > 0 && directions[i] - directions[i - 1] > Math.PI)
                {
                    directions[i] = directions[i] - 2 * Math.PI;
                }

                while (i > 0 && directions[i - 1] - directions[i] > Math.PI)
                {
                    directions[i] = directions[i] + 2 * Math.PI;
                }

                var length = (next - current).Length;
                segmentLengths.Add(length);
            }

            var curvatures = new List<double>();

            for (int i = 0; i < resampledPoints.Count - 2; i++)
            {
                var curvature = (directions[i + 1] - directions[i]) / (segmentLengths[i] + segmentLengths[i + 1]);
                curvatures.Add(curvature);
            }

            var curvatureWasPositive = curvatures[0] > 0;
            var numInflectionPoints = 0;

            for (int i = 1; i < curvatures.Count; i++)
            {
                var currentCurvature = curvatures[i] > 0;

                if (currentCurvature != curvatureWasPositive)
                {
                    Debug.WriteLine("inflection point at " + i/(double)curvatures.Count);
                    numInflectionPoints++;
                    curvatureWasPositive = currentCurvature;
                }
            }

            return numInflectionPoints;
        }

    }
}