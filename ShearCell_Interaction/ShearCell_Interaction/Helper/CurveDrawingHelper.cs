using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using ShearCell_Interaction.Model;

namespace ShearCell_Interaction.Helper
{
    public static class CurveDrawingHelper
    {

        public static List<Vector> GetLine(int numSamples, double scaleFactor, Vector startPoint)
        {
            var vertices = new List<Vector>();
            var step = scaleFactor / numSamples;

            for (int i = 0; i < numSamples; i++)
            {
                var x = -i * step;
                var y = i * step;
                var offset = new Vector(x, y);
                var position = startPoint + offset;
                vertices.Add(position);
            }

            return vertices;
        }

        public static List<Vector> GetCubicPolynomial(int numSamples, double scaleFactor, Vector startPoint)
        {
            var vertices = new List<Vector>();
            var step = 8.0 / numSamples;

            for (int i = 0; i < numSamples; i++)
            {
                var x = -4.37228 + i * step;
                var y = (Math.Pow(x, 3) + Math.Pow(x, 2) - 12 * x + 12) / -12.0;
                var offset = new Vector((x + 4.37228) * scaleFactor, y);
                var position = startPoint + offset;
                vertices.Add(position);
            }

            return vertices;
        }
        public static List<Vector> GetBowCurve(int numSamples, double scaleFactor, Vector startPoint)
        {
            var tempVertices = new List<Vector>();
            var step = 0.75 * Math.PI / numSamples;

            for (double i = numSamples / 2.5; i >= -numSamples / 2.5; i -= 1.0)
            {
                var theta = i * step;

                var r1 = Math.Cos(2 * theta);
                var r2 = Math.Pow(Math.Cos(theta), 2);

                var r = r1 / r2;
                if (Math.Abs(r2) < 0.0001)
                    continue;

                var x = -r * Math.Cos(theta);
                var y = r * Math.Sin(theta);

                var offset = new Vector(x * scaleFactor, y * scaleFactor);
                tempVertices.Add(offset);
            }

            var vertices = new List<Vector>();

            //curve starts at center bottom
            double maxX = tempVertices.Max(current => current.X);
            double minY = tempVertices.Min(current => current.Y);
            var shift = new Vector(maxX, minY);

            foreach (var tempVertex in tempVertices)
            {
                var position = startPoint + tempVertex - shift;
                vertices.Add(position);
            }

            return vertices;
        }

        public static List<Vector> GetTransformedBicorn(int numSamples, double scaleFactor, Vector startPoint)
        {
            var tempVertices = new List<Vector>();
            var step = 2.0 * Math.PI / numSamples;

            for (double i = -numSamples / 2.0; i <= numSamples / 2.0; i += 1.0)
            {
                var theta = i * step;

                var x = scaleFactor * Math.Sin(theta);
                var y = scaleFactor * (Math.Pow(Math.Cos(theta), 2) * (2 + Math.Cos(theta))) /
                           (3.0 + Math.Pow(Math.Sin(theta), 2));


                var offset = new Vector(x * scaleFactor, y * scaleFactor);
                tempVertices.Add(offset);
            }

            var vertices = new List<Vector>();

            //curve starts at center bottom
            double maxX = 0;//tempVertices.Max(current => current.X);
            double minY = scaleFactor / 3.0;//tempVertices.Min(current => current.Y);
            var shift = new Vector(maxX, minY);

            foreach (var tempVertex in tempVertices)
            {
                var position = startPoint + tempVertex - shift;
                vertices.Add(position);
            }

            return vertices;
        }

        public static List<Vector> GetAmpersandCurve(int numSamples, double scaleFactor, Vector startPoint)
        {
            var vertices = new List<Vector>();
            var step = 2 * Math.PI / numSamples;

            for (int i = 0; i <= numSamples; i++)
            {
                var theta = i * step;
                var cosTheta = Math.Cos(theta);
                var cosTheta2 = Math.Cos(2.0 * theta);
                var cosThetaSquared = Math.Pow(cosTheta, 2);

                double r1 = 11 * cosTheta + 10 + Math.Pow(theta, 3) + Math.Sqrt(3) *
                         Math.Sqrt(-1 * (21 * cosThetaSquared - 16) * Math.Pow(2 * cosThetaSquared - 1, 2));
                double r2 = -1 * cosThetaSquared + 2 * Math.Pow(theta, 4) + 2;
                double r = 1.0 / 4.0 * r1 / r2;

                //double r1 = cosTheta * (5.0 * cosTheta2 + 16.0);
                //double r21 = cosThetaSquared * Math.Pow(5 * cosTheta2 + 16.0, 2);
                //double r22 = 8.0 * (16.0 * cosThetaSquared + 3.0 * cosTheta2) * (cosTheta2 * cosThetaSquared + 2.0);

                //double r2 = Math.Sqrt(r21 - r22);
                //double rd = 4.0 * (cosTheta2 * cosThetaSquared + 2.0);

                //double r = (r1 + r2) / rd;

                var x = r * Math.Cos(theta);
                var y = r * Math.Sin(theta);

                var offset = new Vector(x * scaleFactor, y * scaleFactor);
                var position = startPoint + offset;
                vertices.Add(position);
            }

            return vertices;
        }

        public static List<Vector> GetFoliumDescartes(int numSamples, double scaleFactor, Vector startPoint,
            bool reverse = false)
        {
            var tempVertices = new List<Vector>();
            var step = 3.0 / numSamples;

            if (!reverse)
            {
                for (double i = -numSamples / 2.0; i < 0; i++)
                {
                    var t = i * step;
                    double x = t * t - 1;
                    double y = t * (t * t - 1);
                    var offset = new Vector(x, y);
                    tempVertices.Add(offset);
                }

                for (double i = 0; i < numSamples / 2.0; i++)
                {
                    var t = i * step;
                    double x = t * t - 1;
                    double y = t * (t * t - 1);
                    var offset = new Vector(x, y);
                    tempVertices.Add(offset);
                }
            }
            else
            {
                for (double i = numSamples / 2.0; i > 0; i--)
                {
                    var t = i * step;
                    double x = t * t - 1;
                    double y = t * (t * t - 1);
                    var offset = new Vector(x, y);
                    tempVertices.Add(offset);
                }

                for (double i = 0; i > -numSamples / 2.0; i--)
                {
                    var t = i * step;
                    double x = t * t - 1;
                    double y = t * (t * t - 1);
                    var offset = new Vector(x, y);
                    tempVertices.Add(offset);
                }
            }

            Vector shift;

            if (!reverse)
            {
                double minX = tempVertices.Min(current => current.X);
                double minY = tempVertices.Min(current => current.Y);
                shift = new Vector(minX, -minY);
            }
            else
            {
                double maxX = tempVertices.Max(current => current.X);
                double maxY = tempVertices.Max(current => current.Y);
                shift = new Vector(-maxX, -maxY);
            }

            var vertices = new List<Vector>();

            foreach (var tempVertex in tempVertices)
            {
                var offset = tempVertex + shift;
                var position = startPoint + offset * scaleFactor;
                vertices.Add(position);
            }

            return vertices;
        }
    }
}