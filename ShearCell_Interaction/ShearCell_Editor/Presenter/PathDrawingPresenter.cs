using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using ShearCell_Data.Helper;
using ShearCell_Data.View;
using ShearCell_Editor.Event;

namespace ShearCell_Editor.Presenter
{
    public class PathDrawingPresenter
    {
        private const double ARC_RADIUS_LINE_DASH_LENGTH = 5.0;
        private const double ARC_RADIUS_LINE_GAP_LENGTH = 5.0;
        private const int ARC_NUMBER_SAMPLING_POINTS = 100;

        private readonly EditorViewModel _viewModel;
        private readonly InkCanvas _drawingCanvas;

        private bool _isMouseDown;

        private Stroke _currentStraightLineStroke;

        private bool _arcDrawingStarted;
        private List<Stroke> _arcRadiusStrokes;
        private Point _arcRadiusStartPoint;
        private Point _arcRadiusEndPoint;
        private ArcSegment _currenArcSegment;
        private PathGeometry _currentArcGeometry;
        private bool _arcRadiusSet;
        private bool _arcAngleSet;
        private Stroke _currentArcRadiusStroke;

        public delegate void PathCollectedEventHandler(object sender, PathCollectedEventArgs e);
        public event PathCollectedEventHandler OnPathCollected;

        public PathDrawingPresenter(EditorViewModel viewModel, InkCanvas drawingCanvas)
        {
            _viewModel = viewModel;
            _drawingCanvas = drawingCanvas;
            _drawingCanvas.StrokeCollected += OnInkCanvasFreeformStrokeCollected;
        }

        public void HandleMouseDown(Point position)
        {
            _isMouseDown = true;

            if (_viewModel.SelectedElement == ElementSelection.Linear)
                StartStraightLineDrawing(position);

            if (_viewModel.SelectedElement == ElementSelection.Rotation)
            {
                if (!_arcDrawingStarted)
                {
                    StartArcDrawing(position);
                }
                else if (_arcDrawingStarted && _arcRadiusSet && _arcAngleSet)
                {
                    FinalizeDrawingArc();
                }
            }
        }


        public void HandleMouseMove(Point position)
        {
            switch (_viewModel.SelectedElement)
            {
                case ElementSelection.Linear:
                    if (_isMouseDown)
                        UpdateStraightLineDrawing(position);
                    break;
                case ElementSelection.Rotation:
                    if (!_arcDrawingStarted)
                        break;

                    if (!_arcRadiusSet && !_arcAngleSet && _isMouseDown)
                        UpdateArcRadiusLine(position);
                    else if (_arcRadiusSet && !_arcAngleSet && _isMouseDown)
                        UpdateArcLength(position);
                    else if (_arcRadiusSet && _arcAngleSet && !_isMouseDown)
                        UpdateArcScale(position);
                    break;
            }
        }

        public void HandleMouseUp(Point position)
        {
            _isMouseDown = false;

            switch (_viewModel.SelectedElement)
            {
                case ElementSelection.Linear:
                    CheckStraightLineDrawingForValidity();
                    if (_currentStraightLineStroke != null)
                        RaisePathCollectedEvent(_currentStraightLineStroke);
                    break;
                case ElementSelection.Rotation:
                    if (!_arcRadiusSet)
                        _arcRadiusSet = true;
                    else if (_arcRadiusSet && !_arcAngleSet)
                        _arcAngleSet = true;
                    break;
            }
        }

        private void StartStraightLineDrawing(Point position)
        {
            var points = new StylusPointCollection
            {
                new StylusPoint(position.X, position.Y),
                new StylusPoint(position.X, position.Y)
            };
            _currentStraightLineStroke = new Stroke(points, _drawingCanvas.DefaultDrawingAttributes);
            _drawingCanvas.Strokes.Add(_currentStraightLineStroke);
        }

        private void UpdateStraightLineDrawing(Point position)
        {
            _currentStraightLineStroke.StylusPoints[1] = new StylusPoint(position.X, position.Y);
        }

        private void CheckStraightLineDrawingForValidity()
        {
            var points = _currentStraightLineStroke.StylusPoints;
            var distance = (points[0].ToPoint() - points[1].ToPoint()).Length;

            if (distance < 15)
            {
                _drawingCanvas.Strokes.Remove(_currentStraightLineStroke);
                _currentStraightLineStroke = null;
            }
        }

        private void StartArcDrawing(Point position)
        {
            _arcRadiusStrokes = new List<Stroke>();
            Debug.WriteLine("new");

            _arcDrawingStarted = true;
            _arcRadiusSet = false;

            _arcRadiusStartPoint = position;
        }

        private void UpdateArcLength(Point position)
        {
            var center = _arcRadiusStartPoint;
            var outside = _arcRadiusEndPoint;
            var radius = Math.Abs((center - outside).Length);

            var vector1 = outside - center;
            var vector2 = position - center;

            var angle = MathHelper.GetAngleBetweenVectorsInDegree(vector1, vector2);

            var arcDirection = new Vector(vector2.X, vector2.Y);
            arcDirection.Normalize();
            //var arcPosition = center + arcDirection * radius;
            var arcPosition = position;

            //var angle = Math.Asin(radius / hypothenuse) * MathHelper.RadToDeg;

            //Debug.WriteLine(center + " / " + angle.ToString("F") + " / " + radius.ToString("F"));

            var sweepDirection = angle > 0 ? SweepDirection.Clockwise : SweepDirection.Counterclockwise;

            _currenArcSegment = new ArcSegment(arcPosition, new Size(radius, radius), angle, false, sweepDirection, false);
            _currenArcSegment.IsSmoothJoin = false;

            var fig = new PathFigure();
            fig.IsClosed = false;
            fig.IsFilled = false;
            fig.StartPoint = outside;
            fig.Segments.Add(_currenArcSegment);

            _currentArcGeometry = new PathGeometry();
            _currentArcGeometry.Figures.Add(fig);

            var points = GetPointsFromGeometry(_currentArcGeometry);

            if (points.Count <= 0)
            {
                return;
            }

            if (_currentArcRadiusStroke == null)
            {
                var stylusPointCollection = new StylusPointCollection(points);
                _currentArcRadiusStroke = new Stroke(stylusPointCollection, _drawingCanvas.DefaultDrawingAttributes);
                _drawingCanvas.Strokes.Add(_currentArcRadiusStroke);
            }
            else
            {
                UpdateArcStrokeFromPoints(_currentArcRadiusStroke, points);
            }
        }

        private void UpdateArcScale(Point position)
        {
            var arcEnd = _currentArcRadiusStroke.StylusPoints[_currentArcRadiusStroke.StylusPoints.Count - 1].ToPoint();

            var initialRadius = (_arcRadiusStartPoint - _arcRadiusEndPoint).Length;
            var distanceX = arcEnd.X - position.X;
            var radius = Math.Max(0, initialRadius + distanceX);

            _currenArcSegment.Size = new Size(radius, radius);

            var points = GetPointsFromGeometry(_currentArcGeometry);

            if (points.Count <= 0)
            {
                return;
            }

            UpdateArcStrokeFromPoints(_currentArcRadiusStroke, points);
        }

        private void UpdateArcStrokeFromPoints(Stroke stroke, List<Point> points)
        {
            var newPoints = new StylusPointCollection(points);
            stroke.StylusPoints = newPoints;
        }

        private List<Point> GetPointsFromGeometry(PathGeometry geometry, int numberOfPoints = ARC_NUMBER_SAMPLING_POINTS)
        {
            //Geometry is sometimes invalid.
            if (geometry.Bounds.Width < 1 || geometry.Bounds.Height < 1)
                return new List<Point>();

            var points = new List<Point>();
            for (var i = 0; i < numberOfPoints; i++)
            {
                Point point;
                Point tangent;

                geometry.GetPointAtFractionLength(1.0 / (numberOfPoints - 1) * i, out point, out tangent);
                points.Add(point);
            }

            return points;
        }

        private void UpdateArcRadiusLine(Point position)
        {
            foreach (var stroke in _arcRadiusStrokes)
            {
                _drawingCanvas.Strokes.Remove(stroke);
            }
            _arcRadiusStrokes.Clear();

            var distance = (_arcRadiusStartPoint - position).Length;

            var currentDistanceTraveled = 0.0;
            int numberSegments = 0;

            while (currentDistanceTraveled < distance)
            {
                currentDistanceTraveled += ARC_RADIUS_LINE_DASH_LENGTH;

                if (currentDistanceTraveled > distance)
                    break;
                numberSegments++;
                currentDistanceTraveled += ARC_RADIUS_LINE_GAP_LENGTH;
            }

            var direction = position - _arcRadiusStartPoint;
            direction.Normalize();

            for (var i = 0; i < numberSegments; i++)
            {
                var dashStart = _arcRadiusStartPoint + i * (direction * (ARC_RADIUS_LINE_DASH_LENGTH + ARC_RADIUS_LINE_GAP_LENGTH));
                var dashEnd = dashStart + direction * ARC_RADIUS_LINE_DASH_LENGTH;
                var points = new StylusPointCollection
                {
                    new StylusPoint(dashStart.X, dashStart.Y),
                    new StylusPoint(dashEnd.X, dashEnd.Y)
                };

                var dashedStroke = new Stroke(points);
                dashedStroke.DrawingAttributes.FitToCurve = false;
                dashedStroke.DrawingAttributes.Color = Colors.DarkGray;
                _arcRadiusStrokes.Add(dashedStroke);
                _drawingCanvas.Strokes.Add(dashedStroke);
            }

            Debug.WriteLine("filled");
            _arcRadiusEndPoint = position;
        }
        private void FinalizeDrawingArc()
        {
            _arcRadiusSet = false;
            _arcAngleSet = false;
            _arcDrawingStarted = false;

            foreach (var stroke in _arcRadiusStrokes)
            {
                _drawingCanvas.Strokes.Remove(stroke);
            }

            _arcRadiusStrokes.Clear();

            RaisePathCollectedEvent(_currentArcRadiusStroke);

            _currentArcRadiusStroke = null;
        }

        private void OnInkCanvasFreeformStrokeCollected(object sender, InkCanvasStrokeCollectedEventArgs e)
        {
            RaisePathCollectedEvent(e.Stroke);
        }

        private void RaisePathCollectedEvent(Stroke stroke)
        {
            if (OnPathCollected == null)
                return;

            var points = new List<Point>();

            //if (stroke.DrawingAttributes.FitToCurve)
            //    points.AddRange(stroke.GetBezierStylusPoints().Select(stylusPoint => stylusPoint.ToPoint()));
            //else
                points.AddRange(stroke.StylusPoints.Select(stylusPoint => stylusPoint.ToPoint()));

            OnPathCollected(this, new PathCollectedEventArgs(points, stroke));
        }
    }
}