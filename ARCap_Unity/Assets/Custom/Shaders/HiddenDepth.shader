Shader "Unlit/ReadDepth"
{
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            ZWrite Off ColorMask RGB
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            sampler2D _CameraDepthTexture;
            float4 _CameraDepthTexture_TexelSize;

            v2f vert(appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            float LinearizeDepth(float depth)
            {
                float near = _ProjectionParams.z; // Camera near
                float far = _ProjectionParams.w; // Camera far
                return (2.0 * near) / (far + near - depth * (far - near));
            }

            fixed4 frag(v2f i) : SV_Target
            {
                float depth = tex2D(_CameraDepthTexture, i.uv).r;
                depth = LinearizeDepth(depth); // Convert to linear depth
                return fixed4(depth, depth, depth, 1.0);
            }
            ENDCG
        }
    }
}