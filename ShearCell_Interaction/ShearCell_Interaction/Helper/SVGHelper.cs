using System.Collections.Generic;
using System.Windows;
using System.Windows.Documents;
using System.Windows.Media;

namespace ShearCell_Interaction.Helper
{
    public class SVGHelper
    {
        public static void DrawPath(List<Vector> points, string filename, int sideLengthDrawing = 100, int strokeWidth = 2, Color? color = null)
        {
            var exportString = "";
            color = color ?? Colors.Black;

            exportString += "<svg width=\"" + sideLengthDrawing + "\" height=\"" + sideLengthDrawing + "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
            exportString += "<path d=\"M ";

            for (int i = 0; i < points.Count - 1; i++)
            {
                var p1 = points[i];
                var p2 = points[i + 1];

                exportString += p1.X.ToString("F") + " " +
                                p1.Y.ToString("F") + " " +
                                p2.X.ToString("F") + " " +
                                p2.Y.ToString("F") + " ";
            }

            var colorValue = color.Value;
            exportString += "\" stroke-width=\"" + strokeWidth + "\" stroke=\"rgb(" + colorValue.R + "," + colorValue.G + "," + colorValue.B + ")\"  fill=\"none\" />";
            exportString += "</svg>";

            System.IO.File.WriteAllText(filename, exportString);
        }

        public static void DrawPath(List<List<Vector>> listOfPointLists, string filename, List<string> names = null, int sideLengthDrawing = 100, int strokeWidth = 2, string color = "black")
        {
            var exportString = "";

            exportString += "<svg width=\"" + sideLengthDrawing + "\" height=\"" + sideLengthDrawing + "\" xmlns=\"http://www.w3.org/2000/svg\">\n";

            for (var pointsIndex = 0; pointsIndex < listOfPointLists.Count; pointsIndex++)
            {
                var id = "";
                if (names != null)
                    id = names[pointsIndex];

                var points = listOfPointLists[pointsIndex];

                exportString += "<path id=\"" + id + "\" d=\"M ";

                for (int i = 0; i < points.Count - 1; i++)
                {
                    var p1 = points[i];
                    var p2 = points[i + 1];

                    exportString += p1.X.ToString("F") + " " +
                                    p1.Y.ToString("F") + " " +
                                    p2.X.ToString("F") + " " +
                                    p2.Y.ToString("F") + " ";
                }

                exportString += "Z\" stroke-width=\"" + strokeWidth + "\" stroke=\"" + color + "\"  fill=\"none\" />\n";
            }

            exportString += "</svg>";

            System.IO.File.WriteAllText(filename, exportString);
        }

        public static void DrawPath(List<Point> points, string filename, int scaleFactor = 25, int sideLengthDrawing = 100, int strokeWidth = 2, Color? color = null, int offset = 50)
        {
            var pointsAsVectors = new List<Vector>();

            foreach (var point in points)
            {
                var globalPoint = CoordinateConverter.ConvertScreenToGlobalCellCoordinates(new Vector(point.X, point.Y));
                var scaledAndOffsetPoint = globalPoint * scaleFactor;
                scaledAndOffsetPoint.Y = sideLengthDrawing - scaledAndOffsetPoint.Y - offset;
                scaledAndOffsetPoint.X += offset;
                pointsAsVectors.Add(scaledAndOffsetPoint);
            }

            DrawPath(pointsAsVectors, filename, sideLengthDrawing, strokeWidth, color);
        }
    }
}