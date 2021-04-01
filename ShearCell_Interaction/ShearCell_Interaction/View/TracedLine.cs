using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;
using ShearCell_Interaction.Helper;

namespace ShearCell_Interaction.View
{
    public class TracedLine
    {
        private const bool _showBackgroundLine = true;
        private readonly Color _backgroundLineColor = Colors.White;
        private const double _defaultThickness = 3.0;

        protected readonly RangeObservableCollection<Shape> _observableShapes;

        public Polyline BackgroundLine { get; }
        public Polyline Line { get; }
        public Color Color { get; }
        public double Thickness { get; }

        public TracedLine(RangeObservableCollection<Shape> observableShapes, Color color) : this(observableShapes, color, _defaultThickness)
        { }

        public TracedLine(RangeObservableCollection<Shape> observableShapes, Color color, double thickness)
        {
            _observableShapes = observableShapes;
            Color = color;
            Thickness = thickness;

            Line = new Polyline { Stroke = new SolidColorBrush(color), StrokeThickness = thickness, StrokeMiterLimit = 2 };
            BackgroundLine = new Polyline {Stroke = new SolidColorBrush(_backgroundLineColor), StrokeThickness = thickness*3, StrokeMiterLimit = 2 };

            _observableShapes.Add(BackgroundLine);
            _observableShapes.Add(Line);
        }

        public void AddPoint(Point point)
        {
            Line.Points.Add(point);

            if(_showBackgroundLine)
                BackgroundLine.Points.Add(point);
        }

        public void Clear()
        {
            Line.Points.Clear();
            BackgroundLine.Points.Clear();
        }
    }
}
