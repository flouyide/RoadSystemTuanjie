using UnityEditor;
using UnityEngine;

namespace RoadSystem.EditorTools
{
    /// <summary>车道标线 atlas 需要 Repeat 包裹（UV.v 随弧长平铺），导入时自动设置。</summary>
    public class RoadTextureImport : AssetPostprocessor
    {
        void OnPreprocessTexture()
        {
            if (!assetPath.EndsWith("RoadLaneAtlas.png")) return;
            var imp = (TextureImporter)assetImporter;
            imp.wrapMode = TextureWrapMode.Repeat;
            imp.mipmapEnabled = true;
            imp.filterMode = FilterMode.Bilinear;
        }
    }
}
