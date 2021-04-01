using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using ShearCell_Data.Model;

namespace ShearCell_Editor
{
    public class EditorPresenter
    {
        private readonly MetamaterialModel _model;
        private readonly EditorViewModel _viewModel;

        public EditorPresenter(MetamaterialModel model, EditorViewModel viewModel)
        {
            _model = model;
            _viewModel = viewModel;
        }

        public void UpdateBoundary(Point currentScreenPosition)
        {
            //model: add shear cell for all cells marked as boundary
            //_model.AddCell(new ShearCell(), );

            _viewModel.AddCellOnGrid(currentScreenPosition);
        }

        private bool TryGetGridPositionFrom2D(Point screenPosition, out Point3D gridPostion)
        {
            //TODO for 3D, add intersection with other voxels
            //var hits = Viewport.FindHits(screenPosition);

            gridPostion = new Point3D();

            var ray = _viewModel.Viewport.Point2DtoRay3D(screenPosition);
            if (ray == null)
                return false;

            var planeIntersection = ray.PlaneIntersection(new Point3D(0, 0, 0.5), new Vector3D(0, 0, 1));
            if (!planeIntersection.HasValue)
                return false;

            gridPostion = new Point3D(Math.Round(planeIntersection.Value.X), Math.Round(planeIntersection.Value.Y), 0);
            return true;
        }
    }
}
