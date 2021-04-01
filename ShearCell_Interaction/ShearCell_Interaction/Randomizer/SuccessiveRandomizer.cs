using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.View;
using ShearCell_Interaction.Simulation;

namespace ShearCell_Interaction.Randomizer
{
    class SuccessiveRandomizer
    {
        public int MinDoF { get; set; }
        public int MaxDoF { get; set; }

        private Vector _lastAddedCellPosition;
        private bool[] _globalGrowDirections = { true, true, true, true }; //{true, false, true, false}; //pos x, neg x, pos y, neg y
        private Random _random;

        private MetamaterialModel _model;
        private ViewModel _viewModel;

        public SuccessiveRandomizer(MetamaterialModel model, ViewModel viewModel)
        {
            _model = model;
            _viewModel = viewModel;

            _random = new Random(Guid.NewGuid().GetHashCode());

            if (_model.Cells.Count < 1)
            {
                Console.WriteLine("Please add an anchored cell first!");
                return;
            }

            _lastAddedCellPosition = _model.Cells[0].IndexVertex.ToInitialVector();

            MinDoF = 1;
            MaxDoF = 3;
        }

        private delegate Vector? CandidatePositionDelegate(Vector referencePosition);
        private delegate int CellTypeDelegate();


        public void AddNextCell()
        {
            AddNextCell(GetCandidateCloseToAnchor, GetShearBiasedCellType);
        }

        private void AddNextCell(CandidatePositionDelegate candidatePositionDelegate, CellTypeDelegate cellTypeDelegate)
        {
            //var connectedComponents = _model.UpdateConstraintsGraph();
            //var currentDoF = connectedComponents.Count - 1;
            var currentDoF = _model.CurrentDoF - 1;
            Debug.WriteLine("current DoF: {0}", currentDoF);

            var candidate = candidatePositionDelegate(_lastAddedCellPosition);
            if (candidate == null)
            {
                Debug.WriteLine("no empty neighbors. exiting");
                return;
            }

            var position = candidate.Value;

            if (currentDoF < MinDoF)
            {
                _model.AddCell(new ShearCell(), position, new Size(1, 1));
                Debug.WriteLine("   add SHEAR cell at {0}", position);
            }
            else if (currentDoF > MaxDoF)
            {
                //add rigid cell
                //define position where to add: if too many DoF add rigid cell so that it couples two shear cells (corner 2 shear, or shear and rigid)

                _model.AddCell(new RigidCell(), position, new Size(1, 1));
                Debug.WriteLine("   add RIGID cell at {0}", position);
            }
            else
            {
                var randomCellType = cellTypeDelegate();

                Cell cell;
                if (randomCellType == 0)
                    cell = new RigidCell();
                else
                    cell = new ShearCell();

                _model.AddCell(cell, position, new Size(1, 1));
                Debug.WriteLine("   add {1} cell at {0}", position, cell.GetType().ToString());
            }

            _lastAddedCellPosition = position;

            //connectedComponents = _model.GetConstraintsGraph();
            //Debug.WriteLine("new DoF: {0}", connectedComponents.Count - 1);            
            Debug.WriteLine("new DoF: {0}", _model.CurrentDoF - 1);
            Debug.WriteLine("");

            //MarkNonShearingCells(connectedComponents);
            //MarkNonShearingCells(_model.ConnectedComponents);
        }

        private int GetEqualChanceCellType()
        {
            return _random.Next(0, 2);
        }
        private int GetShearBiasedCellType()
        {
            var total = 10;
            var shearBias = 0.9;
            var rigidBias = 1 - shearBias;
            var biasedList = new List<int>(total);

            for (var i = 0; i < shearBias*total; i++)
                biasedList.Add(1);
            for (var i = 0; i < rigidBias*total; i++)
                biasedList.Add(0);

            var randomIndex = _random.Next(0, total);
            return biasedList[randomIndex];
        }

        private Vector? GetCandidateCloseToAnchor(Vector currentPosition)
        {
            var candidatePositions = GetFreeNeighborPositions(currentPosition, new List<int> { -1, 1 }, new List<int> { -1, 1 });
            //var candidatePositions = GetFreeNeighborPositions(currentPosition, new List<int> { 1 }, new List<int> { 1 });
            //var candidatePositions = GetFreeNeighborPositions(currentPosition, new List<int> { 1, 1 }, new List<int> { 1, 1 });
            if (candidatePositions.Count < 1)
                return null;

            var anchor = _model.GetAnchors()[0].ToInitialVector();

            candidatePositions.Sort(
                delegate(Vector current, Vector other)
                {
                    var distanceCurrent = Vector.Subtract(current, anchor).Length;
                    var distanceOther = Vector.Subtract(other, anchor).Length;

                    if (distanceCurrent < distanceOther)
                        return -1;
                    if (distanceCurrent > distanceOther)
                        return 1;

                    return 0;
                }
            );

            return candidatePositions[0];
        }

        private Vector? GetCandidateFromTopRightBottom(Vector currentPosition)
        {
            var candidatePositions = GetFreeNeighborPositions(currentPosition, new List<int> {1}, new List<int> {-1, 1});
            if (candidatePositions.Count < 1)
                return null;

            var positionIndex = _random.Next(0, candidatePositions.Count);
            return candidatePositions[positionIndex];
        }

        private Vector? GetCandidateFrom4Directions(Vector currentPosition)
        {
            var candidatePositions = GetFreeNeighborPositions(currentPosition, new List<int> {-1, 1}, new List<int> {-1, 1});
            if (candidatePositions.Count < 1)
                return null;

            var positionIndex = _random.Next(0, candidatePositions.Count);
            return candidatePositions[positionIndex];
        }

        private List<Vector> GetFreeNeighborPositions(Vector currentPosition, List<int> xDirections, List<int> yDirections)
        {
            var positionsToCheck = new List<Vector>();

            foreach (var xDirection in xDirections)
                positionsToCheck.Add(new Vector(xDirection, 0));

            foreach (var yDirection in yDirections)
                positionsToCheck.Add(new Vector(0, yDirection));


            var freePositions = new List<Vector>();
            foreach (var position in positionsToCheck)
            {
                var gridPosition = Vector.Add(currentPosition, position);

                if (!_model.Cells.Exists(c => c.IndexVertex.ToVector().Equals(gridPosition)))
                    freePositions.Add(gridPosition);
            }

            var anchor = _model.GetAnchors()[0].ToInitialVector();
            var maxX = _globalGrowDirections[0] ? int.MaxValue : anchor.X;
            var minX = _globalGrowDirections[1] ? int.MinValue : anchor.X;
            var maxY = _globalGrowDirections[2] ? int.MaxValue : anchor.Y;
            var minY = _globalGrowDirections[3] ? int.MinValue : anchor.Y;

            //check global constraints
            for (var i = 0; i < freePositions.Count; i++)
            {
                var position = freePositions[i];

                if (position.X > maxX || position.X < minX || position.Y > maxY || position.Y < minY)
                    freePositions.Remove(position);
            }

            return freePositions;
        }

        private void MarkNonShearingCells(List<List<List<Edge>>> connectedComponents)
        {
            _viewModel.DebugItems.Clear();

            var isModelChanged = true;
            while (isModelChanged)
            {
                isModelChanged = false;

                for (var i = 0; i < _model.Cells.Count; i++)
                {
                    var cell = _model.Cells[i];
                    if (cell is RigidCell)
                        continue;

                    foreach (var component in connectedComponents)
                    {
                        var flattend = component.SelectMany(e => e);
                        var intersected = flattend.Intersect(cell.CellEdges).ToList();

                        if (intersected.Count == 4)
                        {
                            var cellPosition = cell.IndexVertex.ToInitialVector();
                            //_model.AddCell(new RigidCell(), cellPosition, new Size(1, 1));
                            //isModelChanged = true;

                            var points = new PointCollection();

                            foreach (var vertex in cell.CellVertices)
                            {
                                var currentVertex = CoordinateConverter.ConvertGlobalToScreenCoordinates(vertex.ToInitialVector());
                                points.Add(new Point(currentVertex.X, currentVertex.Y));
                            }

                            var polygon = new Polygon
                            {
                                Points = points,
                                Fill = Brushes.DarkGray,
                                Opacity = 0.7
                            };
                            _viewModel.DebugItems.Add(polygon);
                        }
                    }
                }
            }
        }

    }
}
