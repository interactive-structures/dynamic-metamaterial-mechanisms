using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;

namespace ShearCell_Interaction.Simulation
{
    public class RigidCell : Cell
    {
        //public RigidCell() : base()
        //{
        //    CellConstraints = new List<HashSet<Edge>> { new HashSet<Edge>(CellEdges) };
        //}

        public override void SetConstraints()
        {
            CellConstraints = new List<HashSet<Edge>> { new HashSet<Edge>(CellEdges) };
        }

        public override bool CanMove()
        {
            var numberAnchors = CellVertices.Count(vertex => vertex.IsAnchor);
            return numberAnchors < 2;
        }

        public override void Deform()
        {
            Debug.IndentLevel = 1;
            Debug.WriteLine("DEFORM() " + this);

            if(!CanMove())
                return;
                //throw new Exception("should propagate movement, but can't move! " + this);

            if (AlreadyDeformedVertexIndices.Count < 1 || AlreadyDeformedVertexIndices.Count > 2)
                return;

            if (AlreadyDeformedVertexIndices.Count == 2)
                RotateCell();
            else 
                TranslateCell();

            CheckEdgesForCoherence();
            AlreadyDeformedVertexIndices.Clear();
        }

        public override void SetInitialDeformedVertices(Vertex movedVertex, Vector targetOffset)
        {
            Debug.IndentLevel = 1;
            Debug.WriteLine("DeformINITIAL() " + this);

            if (movedVertex.IsAnchor)
                return;
            if(!CanMove())
                return;

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
                var anchorIndex = CellVertices.IndexOf(anchors[0]);
                AlreadyDeformedVertexIndices.Add(anchorIndex);

                var isAdjacent = MathHelper.Mod(movedVertexIndex - anchorIndex, 2) == 1;
                var lengthConstraint = isAdjacent
                    ? GetEdgeLengthBetweenVertices(anchorIndex, movedVertexIndex)
                    : GetDiagonalLenth();

                var movedTargetLocal = Vector.Subtract(movedTarget, anchors[0].ToVector());
                movedTargetLocal = MathHelper.SetVectorLength(movedTargetLocal, lengthConstraint);
                movedTarget = Vector.Add(movedTargetLocal, anchors[0].ToVector());

                CellVertices[movedVertexIndex].SetPosition(movedTarget);
                AlreadyDeformedVertexIndices.Add(movedVertexIndex);
            }
        }

        public override double GetMaxDiagonalLength()
        {
            return Math.Sqrt(_edgeLengthCCW * _edgeLengthCCW + _edgeLengthCW * _edgeLengthCW);
        }

        public override string GetEdgeStyle(bool isDeformed)
        {
            return isDeformed ? "DeformedRigidCellStyle" : "InitialRigidCellStyle";
        }

        public override string GetVertexStyle(bool isDeformed)
        {
            return isDeformed ? "DeformedVertexStyle" : "InitialVertexStyle";
        }

        public override string ToString()
        {
            return "Rigid cell at " + IndexVertex;
        }

    }
}
