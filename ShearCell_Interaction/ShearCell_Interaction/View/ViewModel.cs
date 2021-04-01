using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Properties;
using ShearCell_Interaction.Simulation;

namespace ShearCell_Interaction.View
{
    public class ViewModel : INotifyPropertyChanged
    {
        public static int WindowHeight = 800;
        public static int WindowWidth = 1200;

        public static double CellScale = 30; //25.0; //50;
        public static int VertexVisualSize = 2;
        public static bool HideInitialCells = true;

        public RangeObservableCollection<Shape> AnchorShapeItems { get; set; }
        public RangeObservableCollection<Shape> InputShapeItems { get; set; }
        public RangeObservableCollection<Shape> DeformedShapeItems { get; set; }
        public RangeObservableCollection<Shape> TracingVerticesShapeItems { get; set; }
        public RangeObservableCollection<Shape> DebugItems { get; set; }

        private InteractionMode _interactionMode;
        public InteractionMode InteractionMode
        {
            get { return _interactionMode; }
            set
            {
                _interactionMode = value;
                UpdateVisibilities();
                OnPropertyChanged();
            }
        }

        private ElementSelection _selectedElement;
        public ElementSelection SelectedElement
        {
            get { return _selectedElement; }
            set { _selectedElement = value; OnPropertyChanged(); }
        }

        private Visibility _undeformedViewVisibility;
        public Visibility UndeformedViewVisibility
        {
            get { return _undeformedViewVisibility; }
            set { _undeformedViewVisibility = value; OnPropertyChanged(); }
        }

        private Visibility _deformedViewVisibility;
        public Visibility DeformedViewVisibility
        {
            get { return _deformedViewVisibility; }
            set { _deformedViewVisibility = value; OnPropertyChanged(); }
        }

        private Visibility _addCellVisibility;
        public Visibility AddCellVisibility
        {
            get { return _addCellVisibility; }
            set { _addCellVisibility = value; OnPropertyChanged(); }
        }

        private PointCollection _inputPath;
        public PointCollection InputPath
        {
            get { return _inputPath; }
            set
            {
                _inputPath = value;
                OnPropertyChanged();

                if (value != null && _inputPath.Count > 0)
                {
                    InputPathLength = PolylineHelper.PathLength(_inputPath, true);
                    //InputPathLength = PolylineHelper.NumInflectionPoints(Model.InputPath);
                }
            }
        }

        private PointCollection _outputPath;
        public PointCollection OutputPath
        {
            get { return _outputPath; }
            set
            {
                _outputPath = value;
                OnPropertyChanged();

                if (_outputPath != null && _outputPath.Count > 0)
                {
                    OutputPathLength = PolylineHelper.PathLength(_outputPath, true);
                    //InputPathLength = PolylineHelper.NumInflectionPoints(Model.OutputPath);
                }
            }
        }

        private string _currentDoF;
        public string CurrentDoF
        {
            get { return _currentDoF; }
            set { _currentDoF = value; OnPropertyChanged(); }
        }

        private double _inputPathLength = -1;
        public double InputPathLength
        {
            get { return _inputPathLength; }
            set { _inputPathLength = value; OnPropertyChanged(); }
        }

        private double _outputPathLength = -1;
        public double OutputPathLength
        {
            get { return _outputPathLength; }
            set { _outputPathLength = value; OnPropertyChanged(); }
        }

        private double _inputPathDistance = -1;
        public double InputPathDistance
        {
            get { return _inputPathDistance; }
            set { _inputPathDistance = value; OnPropertyChanged(); }
        }

        private double _outputPathDistance = -1;
        public double OutputPathDistance
        {
            get { return _outputPathDistance; }
            set { _outputPathDistance = value; OnPropertyChanged(); }
        }
        public Dictionary<Vertex, TracedLine> TracedVertices
        {
            get { return _tracedVertices; }
        }

        public bool LoopAnimation
        {
            get { return _loopAnimation; }
            set
            {
                _loopAnimation = value;
                OnPropertyChanged();
            }
        }

        public int NumberIterationsGeneration
        {
            get { return _numberIterationsGeneration; }
            set { _numberIterationsGeneration = value; OnPropertyChanged(); }
        }

        public bool UseSimulatedAnnealing
        {
            get { return _useSimulatedAnnealing; }
            set
            {
                _useSimulatedAnnealing = value;
                OnPropertyChanged();
            }
        }


        public int NumPathDivides
        {
            get { return _numPathDivides; }
            set { _numPathDivides = value; OnPropertyChanged(); }
        }

        public int MinNumberPathSamples
        {
            get { return _minNumberPathSamples; }
            set { _minNumberPathSamples = value; OnPropertyChanged(); }
        }
        public int MaxNumberPathSamples
        {
            get { return _maxNumberPathSamples; }
            set { _maxNumberPathSamples = value; OnPropertyChanged(); }
        }
        public int NumScalesOptimization
        {
            get { return _numScalesOptimization; }
            set { _numScalesOptimization = value; OnPropertyChanged(); }
        }

        private bool _scalePathWithCells;
        public bool ScalePathWithCells
        {
            get { return _scalePathWithCells; }
            set
            {
                _scalePathWithCells = value;
                OnPropertyChanged();
            }
        }


        public bool IsDoFVisible { get; set; }
        public bool AreNonShearingCellsMarked { get; set; }
        public bool TracePointsEnabled { get; set; }

        private readonly Dictionary<Vertex, Color> _vertexColors;
        //private readonly Dictionary<Vertex, TracedCutLine> _tracedVertices;
        private readonly Dictionary<Vertex, TracedLine> _tracedVertices;

        private static readonly Color _standardVertexColor = Colors.Black;
        private static readonly Color _inputVertexColor = Colors.LightBlue;
        private static readonly Color _outputVertexColor = Colors.DodgerBlue;

        public MetamaterialModel Model { get; set; }
        private bool _loopAnimation;
        private Dictionary<Vertex, Shape> _vertexElipseDictionary;
        private static Dictionary<Cell, Polygon> _cellPolygonDictionary;

        private int _numberIterationsGeneration;
        private bool _useSimulatedAnnealing;
        private int _numPathDivides;
        private int _minNumberPathSamples;
        private int _numScalesOptimization;
        private int _maxNumberPathSamples;

        #region NotifyPropertyChanged
        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            var handler = PropertyChanged;
            if (handler != null)
                handler(this, new PropertyChangedEventArgs(propertyName));
        }
        #endregion NotifyPropertyChanged

        public ViewModel(MetamaterialModel model)
        {
            Model = model;

            _vertexColors = new Dictionary<Vertex, Color>();
            //_tracedVertices = new Dictionary<Vertex, TracedCutLine>();
            _tracedVertices = new Dictionary<Vertex, TracedLine>();

            AnchorShapeItems = new RangeObservableCollection<Shape>();
            InputShapeItems = new RangeObservableCollection<Shape>();
            DeformedShapeItems = new RangeObservableCollection<Shape>();
            TracingVerticesShapeItems = new RangeObservableCollection<Shape>();
            DebugItems = new RangeObservableCollection<Shape>();

            InteractionMode = InteractionMode.None;

            TracePointsEnabled = true;
            LoopAnimation = true;

            _vertexElipseDictionary = new Dictionary<Vertex, Shape>();
            _cellPolygonDictionary = new Dictionary<Cell, Polygon>();

            NumberIterationsGeneration = 250;
            ScalePathWithCells = true;

            MinNumberPathSamples = 10;
            MaxNumberPathSamples = 40;
            NumPathDivides = 0;

            NumScalesOptimization = 1;
        }

        public void Redraw()
        {
            DrawAnchors(Model.Vertices);
            DrawCells(Model.Cells, true);
            DrawCells(Model.Cells);

            RedrawTracingPoints();
            UpdateVertexColorList();

            CurrentDoF = Model.CurrentDoF.ToString();

            if (IsDoFVisible)
                VisualizeConnectedComponents();

            if (AreNonShearingCellsMarked)
                MarkNonShearingCells();
        }

        public void RedrawDeformed()
        {
            DrawAnchors(Model.Vertices);

            var halfSize = VertexVisualSize * 0.5;

            foreach (var cell in Model.Cells)
            {
                var edgePolygon = _cellPolygonDictionary[cell];
                var edgePoints = new PointCollection();


                foreach (var vertex in cell.CellVertices)
                {
                    var vertexShape = _vertexElipseDictionary[vertex];
                    var screenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(vertex.ToVector());
                    ((Polygon)vertexShape).Points = new PointCollection
                    {
                        new Point(screenPosition.X - halfSize, screenPosition.Y - halfSize),
                        new Point(screenPosition.X + halfSize, screenPosition.Y - halfSize),
                        new Point(screenPosition.X + halfSize, screenPosition.Y + halfSize),
                        new Point(screenPosition.X - halfSize, screenPosition.Y + halfSize)
                    };

                    var currentVertex = CoordinateConverter.ConvertGlobalToScreenCoordinates(vertex.ToVector());
                    edgePoints.Add(new Point(currentVertex.X, currentVertex.Y));
                }

                edgePolygon.Points = edgePoints;
            }

            RedrawTracingPoints();
            UpdateVertexColorList();

            if (IsDoFVisible)
                VisualizeConnectedComponents();

            if (AreNonShearingCellsMarked)
                MarkNonShearingCells();
        }

        public void DrawInputPath()
        {
            PointCollection path;
            DrawPathConstraint(Model.InputPath, out path);

            InputPath = path;
        }

        public void DrawOutputPath()
        {
            PointCollection path;
            DrawPathConstraint(Model.OutputPath, out path);

            OutputPath = path;
        }

        private void DrawPathConstraint(List<Vector> sourcePath, out PointCollection targetPath)
        {
            targetPath = new PointCollection();

            foreach (var vector in sourcePath)
            {
                var screenVector = CoordinateConverter.ConvertGlobalToScreenCoordinates(vector);
                targetPath.Add(new Point(screenVector.X, screenVector.Y));
            }
        }

        public void HighlightInputVertex()
        {
            HighlightVertex(Model.InputVertex, _inputVertexColor);
            //if(_model.InputVertex == null)
            //    return;

            //if (_vertexColors.ContainsValue(_inputVertexColor))
            //{
            //    var oldInputVertex = _vertexColors.FirstOrDefault(x => x.Value == _inputVertexColor).Key;
            //    _vertexColors[oldInputVertex] = _standardVertexColor;
            //}

            //_vertexColors[_model.InputVertex] = _inputVertexColor;
            //DrawCells(_model.Cells);
        }

        public void HighlightOutputVertex()
        {
            HighlightVertex(Model.OutputVertex, _outputVertexColor);
        }

        private void HighlightVertex(Vertex toHighlight, Color highlightColor)
        {
            if (toHighlight == null)
                return;

            if (_vertexColors.ContainsValue(highlightColor))
            {
                var oldInputVertex = _vertexColors.FirstOrDefault(x => x.Value == highlightColor).Key;
                _vertexColors[oldInputVertex] = _standardVertexColor;
            }

            _vertexColors[toHighlight] = highlightColor;
            DrawCells(Model.Cells);
        }

        public void RedrawTracingPoints()
        {
            if (!TracePointsEnabled)
                return;

            foreach (var vertex in _tracedVertices.Keys)
            {
                var screenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(vertex.ToVector());

                var tracedLine = _tracedVertices[vertex];
                tracedLine.AddPoint(new Point(screenPosition.X, screenPosition.Y));
            }
        }

        public void AddTracingPoint(Vertex vertex)
        {
            if (_tracedVertices.ContainsKey(vertex))
                return;

            var numberTracedPoints = _tracedVertices.Count + 1;
            //_tracedVertices.Add(vertex, new TracedCutLine(TracingVerticesShapeItems, Colors.Black));
            _tracedVertices.Add(vertex, new TracedLine(TracingVerticesShapeItems, Colors.Black));

            for (var i = 0; i < numberTracedPoints; i++)
            {
                var hue = i / (double)numberTracedPoints;
                var colorRGB = ColorConversionHelper.ColorFromHSL(hue + 0.3, 1.0, 0.5);
                //var tracedLine = new TracedCutLine(TracingVerticesShapeItems, colorRGB);
                var tracedLine = new TracedLine(TracingVerticesShapeItems, colorRGB);

                var currentVertex = _tracedVertices.Keys.ElementAt(i);
                _vertexColors[currentVertex] = colorRGB;
                _tracedVertices[currentVertex] = tracedLine;
            }

            DrawCells(Model.Cells);
        }

        public void Reset()
        {
            _vertexColors.Clear();
            _tracedVertices.Clear();
            TracingVerticesShapeItems.Clear();

            InputPath = null;
            OutputPath = null;
        }

        public void ResetTracingPaths()
        {
            foreach (var tracedLine in _tracedVertices.Values)
                tracedLine.Clear();
        }

        public void VisualizeConnectedComponents()
        {
            //var connectedComponents = _model.GetConstraintsGraph();

            if (Model.Vertices.Count <= 0)
                return;

            var i = 0;
            //for (var i = 0; i < _model.ConnectedComponents.Count; i++)
            foreach (var component in Model.ConstraintGraph)
            {
                //var component = _model.ConnectedComponents[i].SelectMany(e => e);
                var flattendComponent = component.SelectMany(e => e);

                var hue = i / (double)Model.ConstraintGraph.Count;
                var colorRGB = ColorConversionHelper.ColorFromHSL(hue, 1.0, 0.5);
                var brush = new SolidColorBrush(colorRGB);

                Debug.WriteLine("Connected component [{0}] has hue {1:0.00}.    (0=red, 0.17=yellow, 0.33=green, 0.5=cyan, 0.66=blue, 0.83=magenta)", i, hue);

                foreach (var edge in flattendComponent)
                {
                    var point1 = CoordinateConverter.ConvertGlobalToScreenCoordinates(edge.Vertex1.ToInitialVector());
                    var point2 = CoordinateConverter.ConvertGlobalToScreenCoordinates(edge.Vertex2.ToInitialVector());

                    var polyline = new Polyline
                    {
                        Stroke = brush,
                        Points = { new Point(point1.X, point1.Y), new Point(point2.X, point2.Y) }
                    };

                    if (_deformedViewVisibility == Visibility.Visible)
                        DeformedShapeItems.Add(polyline);
                    else
                        InputShapeItems.Add(polyline);
                }

                i++;
            }
        }

        public void MarkNonShearingCells()
        {
            //DebugItems.Clear();

            for (var i = 0; i < Model.Cells.Count; i++)
            {
                var cell = Model.Cells[i];
                if (cell is RigidCell)
                    continue;

                foreach (var component in Model.ConstraintGraph)
                {
                    var flattend = component.SelectMany(e => e);
                    var intersected = flattend.Intersect(cell.CellEdges).ToList();

                    if (intersected.Count == 4)
                    {
                        var cellPosition = cell.IndexVertex.ToInitialVector();
                        var points = new PointCollection();

                        foreach (var vertex in cell.CellVertices)
                        {
                            var currentVertex = CoordinateConverter.ConvertGlobalToScreenCoordinates(vertex.ToInitialVector());
                            points.Add(new Point(currentVertex.X, currentVertex.Y));
                        }

                        var polygon = new Polygon
                        {
                            Points = points,
                            Fill = Brushes.DarkBlue,
                            Opacity = 0.1
                        };
                        //DebugItems.Add(polygon);
                        InputShapeItems.Add(polygon);
                    }
                }
            }
        }

        private void UpdateVertexColorList()
        {
            if (Model.Vertices.Count == _vertexColors.Count)
                return;

            for (var i = _vertexColors.Count; i < Model.Vertices.Count; i++)
            {
                var vertex = Model.Vertices[i];
                if (_vertexColors.ContainsKey(vertex))
                    continue;

                _vertexColors.Add(vertex, _standardVertexColor);
            }
        }

        private void DrawAnchors(IEnumerable<Vertex> vertices)
        {
            AnchorShapeItems.Clear();
            const string anchorStyleID = "AnchoredVertexStyle";

            foreach (var vertex in vertices)
            {
                if (!vertex.IsAnchor)
                    continue;

                AnchorShapeItems.Add(CreateVertex(vertex.ToVector(), anchorStyleID));
            }
        }

        private void DrawCells(List<Cell> cells, bool initial = false)
        {
            var edges = CreateEdges(cells, initial, initial ? null : _cellPolygonDictionary);
            var vertices = CreateVertices(cells, initial, initial ? null : _vertexElipseDictionary);

            var collection = initial ? InputShapeItems : DeformedShapeItems;

            collection.Clear();
            collection.AddRange(edges);
            collection.AddRange(vertices);
        }

        private void UpdateVisibilities()
        {
            if (InteractionMode == InteractionMode.AddRigidCell || InteractionMode == InteractionMode.AddShearCell)
            {
                UndeformedViewVisibility = Visibility.Visible;
                DeformedViewVisibility = Visibility.Hidden;
                AddCellVisibility = Visibility.Visible;
            }
            else
            {
                DeformedViewVisibility = Visibility.Visible;
                AddCellVisibility = Visibility.Hidden;

                if (HideInitialCells)
                    UndeformedViewVisibility = Visibility.Hidden;
            }
        }

        private static IEnumerable<Shape> CreateEdges(IEnumerable<Cell> cells, bool initial, Dictionary<Cell, Polygon> cache = null)
        {
            var shapes = new List<Shape>();

            if (cache != null)
                _cellPolygonDictionary.Clear();

            foreach (var cell in cells)
            {

                var points = new PointCollection();

                foreach (var vertex in cell.CellVertices)
                {
                    var currentVertex = CoordinateConverter.ConvertGlobalToScreenCoordinates(vertex.ToVector(initial));
                    points.Add(new Point(currentVertex.X, currentVertex.Y));
                }
                var styleID = cell.GetEdgeStyle(!initial);
                var polygon = new Polygon
                {
                    Style = Application.Current.FindResource(styleID) as Style,
                    Points = points,
                    StrokeLineJoin = PenLineJoin.Bevel
                };

                if (cache != null && !cache.ContainsKey(cell))
                {
                    cache.Add(cell, polygon);
                    shapes.Add(polygon);
                }
                else if (cache == null)
                {
                    shapes.Add(polygon);
                }
            }

            return shapes;
        }

        private IEnumerable<Shape> CreateVertices(IEnumerable<Cell> cells, bool initial, Dictionary<Vertex, Shape> cache = null)
        {
            var shapes = new List<Shape>();

            if (cache != null)
                cache.Clear();

            foreach (var cell in cells)
            {
                var vertexStyleID = cell.GetVertexStyle(!initial);

                foreach (var vertex in cell.CellVertices)
                {
                    var vertexShape = CreateVertex(vertex.ToVector(initial), vertexStyleID);

                    if (_vertexColors.ContainsKey(vertex) && !initial)
                    {
                        vertexShape.Fill = new SolidColorBrush(_vertexColors[vertex]);
                        vertexShape.Stroke = new SolidColorBrush(_vertexColors[vertex]);
                        vertexShape.StrokeThickness = 4;
                    }

                    if (cache != null && !_vertexElipseDictionary.ContainsKey(vertex))
                    {
                        cache.Add(vertex, vertexShape);
                        shapes.Add(vertexShape);
                    }
                    else if (cache == null)
                    {
                        shapes.Add(vertexShape);
                    }
                }
            }

            return shapes;
        }

        public static Shape CreateVertex(Vector position, string styleID)
        {
            var halfSize = VertexVisualSize * 0.5;

            var screenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(position);

            var polygon = new Polygon
            {
                Style = Application.Current.FindResource(styleID) as Style,
                Points = { new Point(screenPosition.X - halfSize, screenPosition.Y - halfSize), new Point(screenPosition.X + halfSize, screenPosition.Y - halfSize), new Point(screenPosition.X + halfSize, screenPosition.Y + halfSize), new Point(screenPosition.X - halfSize, screenPosition.Y + halfSize) },
                StrokeLineJoin = PenLineJoin.Bevel
            };

            return polygon;
        }

    }
}
