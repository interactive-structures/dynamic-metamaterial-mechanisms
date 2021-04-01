using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;

namespace ShearCell_Interaction.Simulation
{
    public class DeformationController
    {
        private readonly MetamaterialModel _model;

        private List<Cell> _deformedCells;
        private List<Cell> _toDeformCells;

        //private Vertex _movedVertex;
        private Cell _anchoredCell;
        private Vertex _anchorVertex;
        private Vertex _anchoredCellMoveVertex;

        private Vector _movedVector;
        private Vector _anchoredCellMovedVector;
        private Vector _correctedTarget;
        private Vector _error;

        private const float LengthTolerance = 0.001f;
        private const int MaxIterations = 10;
        private int _iteration;

        public bool IsSimulating { get; private set; }

        public DeformationController(MetamaterialModel model)
        {
            _model = model;

            _deformedCells = new List<Cell>();
            _toDeformCells = new List<Cell>();
        }

        public void Deform(Vector targetOffset)
        {
            IsSimulating = true;

            var cell = _model.InputVertex.ContainingCells[0];
            var anchors = _model.Vertices.FindAll(v => v.IsAnchor);

            var isCellAnchored = cell.CellVertices.Count(v => v.IsAnchor) > 0;
            var isNetAnchored = anchors.Count > 0;

            if (isCellAnchored || !isNetAnchored)
                ForwardDeformation(cell, targetOffset);
            else
                InverseDeformation(targetOffset);

            IsSimulating = false;
        }

        private void ForwardDeformation(Cell cell, Vector targetOffset)
        {
            cell.SetInitialDeformedVertices(_model.InputVertex, targetOffset);
            DeformAllCellsForward(cell);
        }

        private void InverseDeformation(Vector targetOffset)
        {
            DeformAllCellsInversely(targetOffset);
        }

        private void DeformAllCellsForward(Cell cell)
        {
            var isFirst = true;
            var hasNext = true;

            while (hasNext)
            {
                if (!isFirst)
                    UpdateAlreadyDeformedVertices(cell);

                cell.Deform();

                if (cell.IsCollision)
                    return;

                _deformedCells.Add(cell);
                _toDeformCells.Remove(cell);
                UpdateNeighborCellsToDeform(cell);

                hasNext = _toDeformCells.Count > 0;
                if (hasNext)
                    cell = _toDeformCells[0];

                isFirst = false;
            }

            _deformedCells.Clear();
            _toDeformCells.Clear();
        }

        private void UpdateAlreadyDeformedVertices(Cell cell)
        {
            var alreadyDeformedVertexIndices = (from deformCell in _deformedCells
                                                from vertex in deformCell.CellVertices
                                                where vertex.ContainingCells.Contains(cell)
                                                select cell.CellVertices.IndexOf(vertex)
                                               ).Distinct().ToList();

            for (var i = 0; i < 4; i++)
            {
                if (cell.CellVertices[i].IsAnchor && !alreadyDeformedVertexIndices.Contains(i))
                    alreadyDeformedVertexIndices.Add(i);
            }

            cell.AlreadyDeformedVertexIndices = alreadyDeformedVertexIndices;
        }

        private void UpdateNeighborCellsToDeform(Cell cell)
        {
            var neighborCells = new List<Cell>();

            foreach (var cellVertex in cell.CellVertices)
            {
                if (cellVertex.ContainingCells.Count < 2)
                    continue;

                foreach (var containingCell in cellVertex.ContainingCells)
                {
                    if (neighborCells.Contains(containingCell) || _deformedCells.Contains(containingCell) || _toDeformCells.Contains(containingCell))
                        continue;

                    neighborCells.Add(containingCell);
                }
            }

            //neighborCells.Sort(
            //    delegate(Cell currentCell, Cell otherCell)
            //    {
            //        var sharedVerticesCurrent = Vertices.FindAll(x => x.ContainingCells.Contains(currentCell) && x.ContainingCells.Contains(cell)).Count;
            //        var sharedVerticesOther = Vertices.FindAll(x => x.ContainingCells.Contains(otherCell) && x.ContainingCells.Contains(cell)).Count;

            //        if (sharedVerticesCurrent > sharedVerticesOther)
            //            return -1;
            //        if (sharedVerticesCurrent < sharedVerticesOther)
            //            return 1;

            //        return 0;
            //    }
            //);

            neighborCells.Sort(
                delegate (Cell currentCell, Cell otherCell)
                {
                    if (currentCell is RigidCell && otherCell is ShearCell)
                        return -1;
                    if (currentCell is ShearCell && otherCell is RigidCell)
                        return 1;

                    return 0;
                }
            );

            _toDeformCells.AddRange(neighborCells);
        }

        private void DeformAllCellsInversely(Vector targetOffset)
        {
            var movedVertex = _model.InputVertex;
            var anchors = _model.Vertices.FindAll(v => v.IsAnchor);

            _anchorVertex = GetMinDistantVertex(movedVertex, anchors);

            var candidates = (from anchorCellCandidates in _anchorVertex.ContainingCells
                              from moveVertexCandidate in anchorCellCandidates.CellVertices
                              select moveVertexCandidate
                             ).Distinct().ToList();

            _anchoredCellMoveVertex = GetMinDistantVertex(movedVertex, candidates);
            _anchoredCell = _model.Cells.Find(c => c.CellVertices.Contains(_anchorVertex) && c.CellVertices.Contains(_anchoredCellMoveVertex));
            var motionRange = GetMotionRange(_anchoredCell, _anchorVertex);

            Debug.IndentLevel = 1;
            Debug.WriteLine("motion range: {0}", motionRange);


            var target = Vector.Add(movedVertex.ToVector(), targetOffset);
            var targetFromAnchor = Vector.Subtract(target, _anchorVertex.ToVector());

            _correctedTarget = MathHelper.ClampVector(targetFromAnchor, motionRange);
            _movedVector = Vector.Subtract(movedVertex.ToVector(), _anchorVertex.ToVector());
            _anchoredCellMovedVector = Vector.Subtract(_anchoredCellMoveVertex.ToVector(), _anchorVertex.ToVector());
            _error = Vector.Subtract(_movedVector, _correctedTarget);

            FindLength();
            FindRotation();

            //Reset();
        }

        private void FindLength()
        {
            _iteration = 0;
            var errorMotionRange = _correctedTarget.Length - _movedVector.Length;

            var upperBound = _anchoredCellMovedVector;
            var lowerBound = new Vector(0,0);

            if (_movedVector.Length < _correctedTarget.Length)
            {
                upperBound = MathHelper.SetVectorLength(upperBound, _anchoredCell.GetMaxDiagonalLength());
                lowerBound = _anchoredCellMovedVector;
            }

            while (Math.Abs(errorMotionRange) > LengthTolerance && _iteration < MaxIterations)
            {
                var deform = Vector.Subtract(upperBound, lowerBound);
                var targetOffsetFromAnchor = MathHelper.SetVectorLength(deform, deform.Length * 0.5);
                targetOffsetFromAnchor = Vector.Add(lowerBound, targetOffsetFromAnchor);

                var currentAnchoredMoved = Vector.Subtract(_anchoredCellMoveVertex.ToVector(), _anchorVertex.ToVector());
                var targetOffset = Vector.Subtract(targetOffsetFromAnchor, currentAnchoredMoved);

                _anchoredCell.SetInitialDeformedVertices(_anchoredCellMoveVertex, targetOffset);
                DeformAllCellsForward(_anchoredCell);

                _movedVector = Vector.Subtract(_model.InputVertex.ToVector(), _anchorVertex.ToVector());

                if (_movedVector.Length > _correctedTarget.Length)
                    upperBound = targetOffsetFromAnchor;
                else
                    lowerBound = targetOffsetFromAnchor;

                if (Math.Abs(errorMotionRange) < Math.Abs(_correctedTarget.Length - _movedVector.Length))
                {
                    var previousTargetOffset = new Vector(targetOffset.X*-1, targetOffset.Y*-1);
                    Debug.WriteLine("LENGTH: reset deformation");

                    _anchoredCell.SetInitialDeformedVertices(_anchoredCellMoveVertex, previousTargetOffset);
                    DeformAllCellsForward(_anchoredCell);
                    break;
                }

                errorMotionRange = _correctedTarget.Length - _movedVector.Length;
                _iteration++;
            }

            _movedVector = Vector.Subtract(_model.InputVertex.ToVector(), _anchorVertex.ToVector());
            _error = Vector.Subtract(_movedVector, _correctedTarget);
        }

        private void FindRotation()
        {
            _iteration = 0;
            var errorAngle = Vector.AngleBetween(_correctedTarget, _movedVector);

            var horizontal = new Vector(1, 0);
            var movedAngle = Vector.AngleBetween(horizontal, _movedVector);
            var correctedAngle = Vector.AngleBetween(horizontal, _correctedTarget);
            //var upperBound = Vector.AngleBetween(horizontal, _movedVector);
            //var lowerBound = Vector.AngleBetween(horizontal, _correctedTarget);

            //var current = Vector.AngleBetween(horizontal, _anchoredCellMovedVector);
            var currentAnchoredMoved = Vector.Subtract(_anchoredCellMoveVertex.ToVector(), _anchorVertex.ToVector());

            var current = MathHelper.AddVectorAngle(currentAnchoredMoved, (correctedAngle - movedAngle) * MathHelper.DegToRad);
            var targetOffset = Vector.Subtract(current, currentAnchoredMoved);

            _anchoredCell.SetInitialDeformedVertices(_anchoredCellMoveVertex, targetOffset);
            DeformAllCellsForward(_anchoredCell);


            //while (Math.Abs(errorAngle) > LengthTolerance && _iteration < MaxIterations)
            //{


            //    _iteration++;
            //    errorAngle = Vector.AngleBetween(_correctedTarget, _movedVector);
            //}

            _movedVector = Vector.Subtract(_model.InputVertex.ToVector(), _anchorVertex.ToVector());
            _error = Vector.Subtract(_movedVector, _correctedTarget);
        }


        private double GetMotionRange(Cell anchoredCell, Vertex anchorVertex)
        {
            var movedVertex = _model.InputVertex;

            var anchoredCellMoveVertex = GetMinDistantVertex(movedVertex, anchoredCell.CellVertices);
            var motionRange = Vector.Subtract(anchoredCellMoveVertex.ToInitialVector(), anchorVertex.ToInitialVector()).Length;

            var lastVertex = anchoredCellMoveVertex;
            var hasReachedTarget = false;

            while (!hasReachedTarget)
            {
                var candidates = lastVertex.ContainingCells.SelectMany(c => c.CellVertices).Distinct().ToList();
                var nextVertex = GetMinDistantVertex(movedVertex, candidates);

                if (nextVertex.Equals(lastVertex))
                {
                    candidates.Remove(lastVertex);
                    nextVertex = GetMinDistantVertex(movedVertex, candidates);
                }

                if (nextVertex.Equals(movedVertex))
                    hasReachedTarget = true;

                motionRange += Vector.Subtract(nextVertex.ToInitialVector(), lastVertex.ToInitialVector()).Length;
                lastVertex = nextVertex;
            }

            return motionRange;
        }

        private Vertex GetMinDistantVertex(Vertex targetVertex, List<Vertex> candidateVertices)
        {
            var minDistance = Double.PositiveInfinity;
            var minDistantAnchor = candidateVertices[0];

            foreach (var candidate in candidateVertices)
            {
                var distance = Vector.Subtract(targetVertex.ToInitialVector(), candidate.ToInitialVector()).Length;
                if (distance < minDistance)
                {
                    minDistance = distance;
                    minDistantAnchor = candidate;
                }
            }

            return minDistantAnchor;
        }

    }
}
