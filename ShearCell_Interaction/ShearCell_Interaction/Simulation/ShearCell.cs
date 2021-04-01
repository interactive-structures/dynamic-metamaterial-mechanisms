using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;

namespace ShearCell_Interaction.Simulation
{
    public class ShearCell : Cell
    {
        private const float CollisionTolerance = 5; //in degree

        //public ShearCell() : base()
        //{
        //    CellConstraints = new List<HashSet<Edge>> {
        //        new HashSet<Edge> { CellEdges[1], CellEdges[3] },
        //        new HashSet<Edge> { CellEdges[0], CellEdges[2] }
        //    };
        //}

        public override void SetConstraints()
        {
            CellConstraints = new List<HashSet<Edge>> {
                    new HashSet<Edge> { CellEdges[1], CellEdges[3] },
                    new HashSet<Edge> { CellEdges[0], CellEdges[2] }
                };
        }

        public override bool CanMove()
        {
            //TODO number of fixed vertices
            var anchors = CellVertices.FindAll(cell => cell.IsAnchor);

            return anchors.Count < 3 ||
                   anchors.Count == 2 && CellVertices.IndexOf(anchors[0]) + CellVertices.IndexOf(anchors[1])%2 == 1; // 2 anchors, but not diagoanl from each other
        }

        public override void Deform()
        {
            Debug.IndentLevel = 1;
            Debug.WriteLine("DEFORM() " + this);

            if(!CanMove())
                return;
                //throw new Exception("should propagate movement, but can't move! " + this);

            if (AlreadyDeformedVertexIndices.Count < 1 || AlreadyDeformedVertexIndices.Count > 3)
            {
                AlreadyDeformedVertexIndices.Clear();
                return;
            }

            if (AlreadyDeformedVertexIndices.Count == 1)
            {
                TranslateCell();

                AlreadyDeformedVertexIndices.Clear();
                return;
            }

            var isAdjacent = MathHelper.Mod(AlreadyDeformedVertexIndices[1] - AlreadyDeformedVertexIndices[0], 2) == 1;
            if (AlreadyDeformedVertexIndices.Count == 2 && isAdjacent) 
            {
                RotateCell();

                AlreadyDeformedVertexIndices.Clear();
                return;
            }

            var fixedIndex = AlreadyDeformedVertexIndices[0];
            var diagonalIndex = AlreadyDeformedVertexIndices[1];

            if (AlreadyDeformedVertexIndices.Count == 3)
            {
                var undeformedIndex = 0;
                while (AlreadyDeformedVertexIndices.Contains(undeformedIndex))
                    undeformedIndex++;

                diagonalIndex = undeformedIndex;
                fixedIndex = MathHelper.Mod(undeformedIndex + 2, 4);
            }

            var fixedVertex = CellVertices[fixedIndex];
            var fixedEdgeCCW = Vector.Subtract(CellVertices[MathHelper.Mod(fixedIndex + 1, 4)].ToVector(), fixedVertex.ToVector());
            var fixedEdgeCW = Vector.Subtract(CellVertices[MathHelper.Mod(fixedIndex - 1, 4)].ToVector(), fixedVertex.ToVector());

            var diagonal = Vector.Add(fixedEdgeCW, fixedEdgeCCW);
            CellVertices[diagonalIndex].SetPosition(Vector.Add(fixedVertex.ToVector(), diagonal));

            CheckEdgesForCoherence();
            AlreadyDeformedVertexIndices.Clear();
        }

        public override void SetInitialDeformedVertices(Vertex movedVertex, Vector targetOffset)
        {
            Debug.IndentLevel = 1;
            Debug.WriteLine("DeformINITIAL() " + this);

            if(movedVertex.IsAnchor)
                return;
            if(!CanMove())
                return;

            IsCollision = false;

            var anchors = CellVertices.FindAll(cell => cell.IsAnchor);
            var movedVertexIndex = CellVertices.IndexOf(movedVertex);
            var movedTarget = Vector.Add(movedVertex.ToVector(), targetOffset);

            if (anchors.Count == 0)
            {
                CellVertices[movedVertexIndex].SetPosition(movedTarget);
                AlreadyDeformedVertexIndices.Add(movedVertexIndex);
            }
            else if (anchors.Count == 1)
            {
                var anchorVertex = anchors[0];
                var anchorIndex = CellVertices.IndexOf(anchorVertex);
                AlreadyDeformedVertexIndices.Add(anchorIndex);

                var isAdjacent = MathHelper.Mod(movedVertexIndex - anchorIndex, 2) == 1;
                var lengthConstraint = isAdjacent
                    ? GetEdgeLengthBetweenVertices(anchorIndex, movedVertexIndex)
                    : GetMaxDiagonalLength(); // difference to rigid cell

                var movedTargetLocal = Vector.Subtract(movedTarget, anchorVertex.ToVector());
                movedTargetLocal = MathHelper.ClampVector(movedTargetLocal, lengthConstraint); // difference to rigid cell
                movedTarget = Vector.Add(movedTargetLocal, anchorVertex.ToVector());

                CalculateVerticesFromDiagonal(anchorVertex, movedTargetLocal);

                CellVertices[movedVertexIndex].SetPosition(movedTarget);
                AlreadyDeformedVertexIndices.Add(movedVertexIndex);
            }
            else if (anchors.Count == 2)
            {
                AlreadyDeformedVertexIndices = anchors.Select(anchor => CellVertices.IndexOf(anchor)).ToList();

                var anchorIndex = (movedVertexIndex + 2) % 4;
                var anchorVertex = CellVertices[anchorIndex];
                anchors.Remove(anchorVertex);

                var adjacentAnchorVertex = anchors[0];
                var adjacentAnchorIndex = CellVertices.IndexOf(adjacentAnchorVertex);

                var lengthConstraint = GetEdgeLengthBetweenVertices(adjacentAnchorIndex, movedVertexIndex);
                var movedEdge = Vector.Subtract(movedTarget, adjacentAnchorVertex.ToVector());
                movedEdge = MathHelper.SetVectorLength(movedEdge, lengthConstraint);

                CheckForCollision(movedEdge, movedVertex.ToInitialVector(), anchorVertex, adjacentAnchorVertex);
                if(IsCollision)
                    return;

                movedTarget = Vector.Add(adjacentAnchorVertex.ToVector(), movedEdge);
                CellVertices[movedVertexIndex].SetPosition(movedTarget);
                AlreadyDeformedVertexIndices.Add(movedVertexIndex);
            }
        }

        public override double GetMaxDiagonalLength()
        {
            return _edgeLengthCCW + _edgeLengthCW;
        }

        public override string GetEdgeStyle(bool isDeformed)
        {
            return isDeformed ? "DeformedEdgeStyle" : "InitialEdgeStyle";
        }

        public override string GetVertexStyle(bool isDeformed)
        {
            return isDeformed ? "DeformedVertexStyle" : "InitialVertexStyle";
        }

        public override string ToString()
        {
            return "Shear cell at " + IndexVertex;
        }

        private void CheckForCollision(Vector movedEdge, Vector initialMovedEdge, Vertex anchorVertex, Vertex adjacentAnchorVertex)
        {
            var anchoredEdge = Vector.Subtract(adjacentAnchorVertex.ToVector(), anchorVertex.ToVector());
            var movedInitial = Vector.Subtract(initialMovedEdge, adjacentAnchorVertex.ToVector());

            var angle = Vector.AngleBetween(anchoredEdge, movedEdge);
            var originalAngle = Vector.AngleBetween(anchoredEdge, movedInitial);
            Debug.WriteLine("original angle: {0},  angle: {1}, ---- {2}", originalAngle, angle, angle - originalAngle);

            if (angle - originalAngle < -90 + CollisionTolerance || angle - originalAngle > 90 - CollisionTolerance)
            {
                IsCollision = true;
                AlreadyDeformedVertexIndices.Clear();
            }
        }
    }
}
