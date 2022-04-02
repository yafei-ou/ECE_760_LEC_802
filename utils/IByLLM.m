function IbyLlm = IByLLM(Lvec, m, rbyml)
    IbyLlm = IVec2Tensor(Lvec) - m * vec2so3(rbyml).' * vec2so3(rbyml);
end