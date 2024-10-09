Shader "Unlit/DepthGradient"
{
    Properties
    {
        _MainTex ("Main Texture", 2D) = "black" {}
        _MinDistance ("Min Distance", Float) = 0.0
        _MaxDistance ("Max Distance", Float) = 8.0
    }
    // URP SubShader
    SubShader
    {
        PackageRequirements
        {
            "com.unity.render-pipelines.universal": "12.0"
        }

        Tags
        {
            "Queue" = "Geometry"
            "RenderType" = "Opaque"
            "ForceNoShadowCasting" = "True"
            "RenderPipeline" = "UniversalPipeline"
        }

        Pass
        {
            Cull Off
            ZTest Always
            ZWrite Off
            Lighting Off
            LOD 100
            Tags
            {
                "LightMode" = "UniversalForward"
            }


            HLSLPROGRAM

            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            #define real half
            #define real3 half3
            #define real4 half4

            struct appdata
            {
                float3 position : POSITION;
                float2 texcoord : TEXCOORD0;

                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 position : SV_POSITION;
                float2 texcoord : TEXCOORD0;

                UNITY_VERTEX_OUTPUT_STEREO
            };

            CBUFFER_START(DisplayRotationPerFrame)
            float4x4 _DisplayRotationPerFrame;
            CBUFFER_END


            v2f vert (appdata v)
            {
                v2f o;

                UNITY_SETUP_INSTANCE_ID(v);
                ZERO_INITIALIZE(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

                o.position = TransformObjectToHClip(v.position);
                o.texcoord = mul(float3(v.texcoord, 1.0f), _DisplayRotationPerFrame).xy;
                return o;
            }


            real3 HSVtoRGB(real3 arg1)
            {
                real4 K = real4(1.0h, 2.0h / 3.0h, 1.0h / 3.0h, 3.0h);
                real3 P = abs(frac(arg1.xxx + K.xyz) * 6.0h - K.www);
                return arg1.z * lerp(K.xxx, saturate(P - K.xxx), arg1.y);
            }

            TEXTURE2D_FLOAT(_MainTex);
            SAMPLER(sampler_MainTex);

            real _MinDistance;
            real _MaxDistance;

            float4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);

                // Sample the environment depth (in meters).
                float envDistance = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.texcoord).r;

                real lerpFactor = (envDistance - _MinDistance) / (_MaxDistance - _MinDistance);
                real hue = lerp(0.70h, -0.15h, saturate(lerpFactor));
                if (hue < 0.0h)
                {
                    hue += 1.0h;
                }
                real3 color = real3(hue, 0.9h, 0.6h);
                return float4(HSVtoRGB(color), 1.0h);
            }

            ENDHLSL
        }
    }
}
