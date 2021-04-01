using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;
using ShearCell_Interaction.Helper;

namespace ShearCell_Interaction.View
{
    public class TracedCutLine
    {
        private readonly Color _backgroundLineColor = Colors.White;

        private bool _isFirst = true;
        private bool _isFirstAdded = false;
        private readonly RangeObservableCollection<Shape> _observableShapes;

        public List<Line> BackgroundLine { get; }
        public List<Line> Line { get; }
        public Color Color { get; }
        public double Thickness { get; }

        public TracedCutLine(RangeObservableCollection<Shape> observableShapes, Color color) : this(observableShapes, color, 1.0)
        { }

        public TracedCutLine(RangeObservableCollection<Shape> observableShapes, Color color, double thickness)
        {
            _observableShapes = observableShapes;
            Color = color;
            Thickness = thickness;

            Line = new List<Line>();
            BackgroundLine = new List<Line>();
        }

        public void AddPoint(Point point)
        {
            if (_isFirst)
            {
                if (_isFirstAdded)
                    CompleteFirstLine(point);
                else
                    AddFirstLine(point);
            }
            else
            {
                var lastLine = Line[Line.Count - 1];
                var lastPoint = new Point(lastLine.X2, lastLine.Y2);

                var line = CreateLine(lastPoint, point, false);
                Line.Add(line);

                var background = CreateLine(lastPoint, point, true);
                BackgroundLine.Add(background);

                _observableShapes.Add(background);
                _observableShapes.Add(line);
            }
        }

        public void Clear()
        {
            Line.Clear();
            BackgroundLine.Clear();
        }

        private void AddFirstLine(Point point)
        {
            var line = CreateLine(point, point, false);
            Line.Add(line);

            var background = CreateLine(point, point, true);
            BackgroundLine.Add(background);

            _observableShapes.Add(background);
            _observableShapes.Add(line);

            _isFirstAdded = true;
        }

        private void CompleteFirstLine(Point point)
        {
            var segment = Line[Line.Count - 1];
            segment.X2 = point.X;
            segment.Y2 = point.Y;

            var backgroundSegment = BackgroundLine[Line.Count - 1];
            backgroundSegment.X2 = point.X;
            backgroundSegment.Y2 = point.Y;

            _isFirst = false;
        }

        private Line CreateLine(Point start, Point end, bool IsBackground)
        {
            var color = IsBackground ? _backgroundLineColor : Color;
            var thickness = IsBackground ? Thickness * 4 : Thickness;
            //var cap = IsBackground ? PenLineCap.Flat : PenLineCap.Flat;

            var line = new Line
            {
                X1 = start.X,
                Y1 = start.Y,
                X2 = end.X,
                Y2 = end.Y,
                Stroke = new SolidColorBrush(color),
                StrokeThickness = thickness,
                //StrokeStartLineCap = cap,
                //StrokeEndLineCap = cap
            };
            return line;
        }
    }
}
