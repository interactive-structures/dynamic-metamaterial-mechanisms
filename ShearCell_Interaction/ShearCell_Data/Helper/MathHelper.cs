using System;
using System.Windows;

namespace ShearCell_Data.Helper
{
    public class MathHelper
    {
        public const double EPSILON = 0.000001;

        public const double DegToRad = Math.PI / 180;
        public const double RadToDeg = 180 / Math.PI;

        public static int Mod(int x, int m)
        {
            var remainder = x % m;
            return remainder < 0 ? remainder + m : remainder;
        }

        public static bool IsEqualDouble(double number1, double number2)
        {
            return Math.Abs(number1 - number2) < EPSILON;
        }

        public static bool IsUnequalDouble(double number1, double number2)
        {
            return Math.Abs(number1 - number2) > EPSILON;
        }

        public static Vector ClampVector(Vector vector, double maxLength)
        {
            if (vector.Length <= maxLength)
                return vector;

            vector.Normalize();
            return Vector.Multiply(vector, maxLength);
        }

        public static Vector SetVectorLength(Vector vector, double length)
        {
            vector.Normalize();
            vector = Vector.Multiply(vector, length);

            return vector;
        }

        public static Vector SetVectorAngle(Vector vector, double angleRad)
        {
            var length = vector.Length;

            return new Vector(
                        length * Math.Cos(angleRad),
                        length * Math.Sin(angleRad)
                    );
        }

        public static Vector AddVectorAngle(Vector vector, double angleRad)
        {
            var length = vector.Length;
            var currentAngle = Math.Atan2(vector.Y, vector.X);
            var newAngle = currentAngle + angleRad;

            return new Vector(
                        length * Math.Cos(newAngle),
                        length * Math.Sin(newAngle)
                    );
        }

        //Copied from https://stackoverflow.com/questions/13458992/angle-between-two-vectors-2d
        public static double GetAngleBetweenVectorsInDegree(Vector vector1, Vector vector2)
        {
            double sin = vector1.X * vector2.Y - vector2.X * vector1.Y;
            double cos = vector1.X * vector2.X + vector1.Y * vector2.Y;

            return Math.Atan2(sin, cos) * RadToDeg;
        }

        public static double DistanceBetweenPoints(Point p1, Point p2)
        {
            //Y value -1 because of weird WPF coordinate system
            return (p1.X - p2.X) + (p1.Y - p2.Y) * -1.0;
        }
    }
}
