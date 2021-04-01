using System.Collections.Generic;
using System.Windows;
using System.Windows.Ink;

namespace ShearCell_Editor.Event
{
    public class PathCollectedEventArgs
    {
        public List<Point> PathPoints { get; set; }
        public Stroke Stroke { get; }

        public PathCollectedEventArgs(List<Point> pathPoints, Stroke stroke)
        {
            PathPoints = pathPoints;
            Stroke = stroke;
        }
    }
}