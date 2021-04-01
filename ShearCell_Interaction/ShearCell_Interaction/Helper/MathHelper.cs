using System;
using System.Windows;

namespace ShearCell_Interaction.Helper
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
    }
}
